#include "systems/raruco_detector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/value.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace uav_delivery {
namespace systems {
namespace {

using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrixd;
using drake::systems::sensors::ImageRgba8U;

constexpr int kQuadrotorStateSize = 18;
constexpr int kMovingTargetStateSize = 9;

struct DetectionComputation {
  bool projected_in_front{false};
  bool enough_area{false};
  bool enough_alignment{false};
  bool enough_in_frame{false};
  bool detected{false};
  double polygon_area_px{0.0};
  double normal_alignment{0.0};
  double in_frame_fraction{0.0};
  double mean_abs_error{255.0};
  double confidence{0.0};
  Eigen::Vector3d position_C{Eigen::Vector3d::Zero()};
  Eigen::Vector2d center_px{Eigen::Vector2d::Zero()};
  std::array<Eigen::Vector2d, 4> corners_px{};
};

Eigen::MatrixXi BaseMarkerBits() {
  Eigen::MatrixXi bits(4, 4);
  bits << 1, 1, 0, 1,
          0, 0, 1, 1,
          1, 0, 1, 0,
          0, 1, 0, 0;
  return bits;
}

Eigen::MatrixXi AddBorder(const Eigen::MatrixXi& input, int pad, int value = 1) {
  if (pad <= 0) {
    return input;
  }
  Eigen::MatrixXi output =
      Eigen::MatrixXi::Constant(input.rows() + 2 * pad, input.cols() + 2 * pad, value);
  output.block(pad, pad, input.rows(), input.cols()) = input;
  return output;
}

Eigen::MatrixXi BuildRecursiveMarkerBits(int depth,
                                         int inner_borders,
                                         bool external_border) {
  Eigen::MatrixXi current = AddBorder(BaseMarkerBits(), inner_borders - 1, 1);
  const Eigen::MatrixXi base = BaseMarkerBits();

  for (int level = 1; level < depth; ++level) {
    Eigen::MatrixXi next =
        Eigen::MatrixXi::Constant(base.rows() * current.rows(),
                                  base.cols() * current.cols(), 1);
    const Eigen::MatrixXi inverted =
        current.unaryExpr([](int value) { return 1 - value; });
    for (int r = 0; r < base.rows(); ++r) {
      for (int c = 0; c < base.cols(); ++c) {
        next.block(r * current.rows(), c * current.cols(), current.rows(), current.cols()) =
            base(r, c) ? current : inverted;
      }
    }
    current = next;
  }

  if (external_border) {
    current = AddBorder(current, 1, 1);
  }
  return current;
}

std::vector<uint8_t> UpsampleBinaryBits(const Eigen::MatrixXi& bits, int resolution) {
  const int rows = bits.rows();
  const int cols = bits.cols();
  std::vector<uint8_t> image(resolution * resolution, 255);
  for (int y = 0; y < resolution; ++y) {
    const int src_r = std::min(rows - 1, (y * rows) / resolution);
    for (int x = 0; x < resolution; ++x) {
      const int src_c = std::min(cols - 1, (x * cols) / resolution);
      image[y * resolution + x] = bits(src_r, src_c) ? 255 : 0;
    }
  }
  return image;
}

std::vector<uint8_t> RotateSquareImage90(const std::vector<uint8_t>& image, int size) {
  std::vector<uint8_t> rotated(size * size, 255);
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      rotated[y * size + x] = image[(size - 1 - x) * size + y];
    }
  }
  return rotated;
}

RigidTransformd MakeDronePoseFromState(const Eigen::VectorXd& quadrotor_state) {
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_WQ(
      quadrotor_state.data() + 6);
  return RigidTransformd(RotationMatrixd(R_WQ), quadrotor_state.segment<3>(0));
}

RigidTransformd MakeTargetPoseFromState(const Eigen::VectorXd& moving_target_state) {
  return RigidTransformd(
      RollPitchYaw<double>(0.0, 0.0, moving_target_state(2)),
      Eigen::Vector3d(moving_target_state(0), moving_target_state(1), 0.0));
}

double PolygonAreaPx(const std::array<Eigen::Vector2d, 4>& corners) {
  double area2 = 0.0;
  for (int i = 0; i < 4; ++i) {
    const auto& a = corners[i];
    const auto& b = corners[(i + 1) % 4];
    area2 += a.x() * b.y() - b.x() * a.y();
  }
  return std::abs(area2) * 0.5;
}

Eigen::Matrix3d ComputeHomographySquareToQuad(
    const std::array<Eigen::Vector2d, 4>& quad,
    double size) {
  Eigen::Matrix<double, 8, 8> A;
  Eigen::Matrix<double, 8, 1> b;
  const std::array<Eigen::Vector2d, 4> src{{
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(size - 1.0, 0.0),
      Eigen::Vector2d(size - 1.0, size - 1.0),
      Eigen::Vector2d(0.0, size - 1.0),
  }};

  for (int i = 0; i < 4; ++i) {
    const double x = src[i].x();
    const double y = src[i].y();
    const double u = quad[i].x();
    const double v = quad[i].y();
    A.row(2 * i) << x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y;
    A.row(2 * i + 1) << 0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y;
    b(2 * i) = u;
    b(2 * i + 1) = v;
  }

  const Eigen::Matrix<double, 8, 1> h = A.fullPivLu().solve(b);
  Eigen::Matrix3d H;
  H << h(0), h(1), h(2),
       h(3), h(4), h(5),
       h(6), h(7), 1.0;
  return H;
}

uint8_t BilinearGrayAt(const ImageRgba8U& image, double x, double y, bool* in_frame) {
  if (image.width() <= 1 || image.height() <= 1 || x < 0.0 || y < 0.0 ||
      x >= image.width() - 1 || y >= image.height() - 1) {
    *in_frame = false;
    return 255;
  }

  *in_frame = true;
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, image.width() - 1);
  const int y1 = std::min(y0 + 1, image.height() - 1);
  const double tx = x - x0;
  const double ty = y - y0;

  auto gray = [&image](int px, int py) {
    const uint8_t* rgba = image.at(px, py);
    return 0.299 * rgba[0] + 0.587 * rgba[1] + 0.114 * rgba[2];
  };

  const double g00 = gray(x0, y0);
  const double g10 = gray(x1, y0);
  const double g01 = gray(x0, y1);
  const double g11 = gray(x1, y1);
  const double g0 = (1.0 - tx) * g00 + tx * g10;
  const double g1 = (1.0 - tx) * g01 + tx * g11;
  return static_cast<uint8_t>(std::clamp((1.0 - ty) * g0 + ty * g1, 0.0, 255.0));
}

std::vector<uint8_t> WarpProjectedMarker(const ImageRgba8U& image,
                                         const std::array<Eigen::Vector2d, 4>& corners,
                                         int resolution,
                                         double* in_frame_fraction) {
  const Eigen::Matrix3d H = ComputeHomographySquareToQuad(corners, resolution);
  std::vector<uint8_t> warped(resolution * resolution, 255);
  int in_frame_count = 0;
  for (int y = 0; y < resolution; ++y) {
    for (int x = 0; x < resolution; ++x) {
      const Eigen::Vector3d sample = H * Eigen::Vector3d(x + 0.5, y + 0.5, 1.0);
      const double u = sample.x() / sample.z();
      const double v = sample.y() / sample.z();
      bool in_frame = false;
      warped[y * resolution + x] = BilinearGrayAt(image, u, v, &in_frame);
      if (in_frame) {
        ++in_frame_count;
      }
    }
  }
  *in_frame_fraction = static_cast<double>(in_frame_count) /
                       static_cast<double>(resolution * resolution);
  return warped;
}

std::pair<double, double> CompareMarker(const std::vector<uint8_t>& observed,
                                        const std::vector<uint8_t>& expected,
                                        int resolution) {
  std::vector<uint8_t> rotated = expected;
  double best_mae = std::numeric_limits<double>::infinity();
  for (int rotation = 0; rotation < 4; ++rotation) {
    double total = 0.0;
    for (int i = 0; i < resolution * resolution; ++i) {
      total += std::abs(static_cast<double>(observed[i]) - static_cast<double>(rotated[i]));
    }
    best_mae = std::min(best_mae, total / static_cast<double>(resolution * resolution));
    rotated = RotateSquareImage90(rotated, resolution);
  }
  const double confidence = std::clamp(1.0 - best_mae / 255.0, 0.0, 1.0);
  return {best_mae, confidence};
}

void SetPixel(ImageRgba8U* image, int x, int y, const std::array<uint8_t, 4>& rgba) {
  if (x < 0 || y < 0 || x >= image->width() || y >= image->height()) {
    return;
  }
  uint8_t* pixel = image->at(x, y);
  pixel[0] = rgba[0];
  pixel[1] = rgba[1];
  pixel[2] = rgba[2];
  pixel[3] = rgba[3];
}

void CopyImage(const ImageRgba8U& input, ImageRgba8U* output) {
  output->resize(input.width(), input.height());
  for (int y = 0; y < input.height(); ++y) {
    for (int x = 0; x < input.width(); ++x) {
      const uint8_t* src = input.at(x, y);
      uint8_t* dst = output->at(x, y);
      for (int channel = 0; channel < 4; ++channel) {
        dst[channel] = src[channel];
      }
    }
  }
}

void DrawLine(ImageRgba8U* image,
              const Eigen::Vector2d& a,
              const Eigen::Vector2d& b,
              const std::array<uint8_t, 4>& rgba) {
  const int steps = std::max(1, static_cast<int>(std::ceil((b - a).norm())));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector2d p = (1.0 - t) * a + t * b;
    SetPixel(image, static_cast<int>(std::lround(p.x())),
             static_cast<int>(std::lround(p.y())), rgba);
  }
}

void DrawCross(ImageRgba8U* image,
               const Eigen::Vector2d& center,
               int half_extent,
               const std::array<uint8_t, 4>& rgba) {
  for (int dx = -half_extent; dx <= half_extent; ++dx) {
    SetPixel(image, static_cast<int>(std::lround(center.x())) + dx,
             static_cast<int>(std::lround(center.y())), rgba);
  }
  for (int dy = -half_extent; dy <= half_extent; ++dy) {
    SetPixel(image, static_cast<int>(std::lround(center.x())),
             static_cast<int>(std::lround(center.y())) + dy, rgba);
  }
}

DetectionComputation RunDetection(const DroneCameraRenderParams& camera_params,
                                  const RArucoDetectorParams& detector_params,
                                  const RigidTransformd& X_BC,
                                  const std::vector<uint8_t>& expected_marker,
                                  const Eigen::VectorXd& quadrotor_state,
                                  const Eigen::VectorXd& moving_target_state,
                                  const ImageRgba8U& image) {
  DetectionComputation result;

  const RigidTransformd X_WQ = MakeDronePoseFromState(quadrotor_state);
  const RigidTransformd X_WT = MakeTargetPoseFromState(moving_target_state);
  const RigidTransformd X_TM(
      RollPitchYaw<double>(detector_params.marker.rpy), detector_params.marker.position);
  const RigidTransformd X_WC = X_WQ * X_BC;
  const RigidTransformd X_CM = X_WC.inverse() * X_WT * X_TM;

  result.position_C = X_CM.translation();
  result.projected_in_front = result.position_C.z() > camera_params.near;

  const double half_size = 0.5 * detector_params.marker.size_m;
  const std::array<Eigen::Vector3d, 4> marker_corners_M{{
      Eigen::Vector3d(-half_size, -half_size, 0.0),
      Eigen::Vector3d(half_size, -half_size, 0.0),
      Eigen::Vector3d(half_size, half_size, 0.0),
      Eigen::Vector3d(-half_size, half_size, 0.0),
  }};

  const double focal = 0.5 * static_cast<double>(camera_params.width) /
                       std::tan(0.5 * camera_params.fov_deg * M_PI / 180.0);
  const double cx = 0.5 * static_cast<double>(camera_params.width - 1);
  const double cy = 0.5 * static_cast<double>(camera_params.height - 1);

  bool all_corners_in_front = true;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d p_C = X_CM * marker_corners_M[i];
    all_corners_in_front = all_corners_in_front && (p_C.z() > camera_params.near);
    result.corners_px[i] = Eigen::Vector2d(cx + focal * p_C.x() / p_C.z(),
                                           cy + focal * p_C.y() / p_C.z());
  }
  result.projected_in_front = result.projected_in_front && all_corners_in_front;
  result.center_px = 0.25 * (result.corners_px[0] + result.corners_px[1] +
                             result.corners_px[2] + result.corners_px[3]);

  const Eigen::Vector3d normal_C = X_CM.rotation() * Eigen::Vector3d::UnitZ();
  result.normal_alignment = std::max(0.0, -normal_C.normalized().z());
  result.enough_alignment =
      result.normal_alignment >= detector_params.min_normal_alignment;

  result.polygon_area_px = PolygonAreaPx(result.corners_px);
  result.enough_area = result.polygon_area_px >= detector_params.min_marker_area_px;

  double in_frame_fraction = 0.0;
  const std::vector<uint8_t> warped = WarpProjectedMarker(
      image, result.corners_px, detector_params.warp_resolution, &in_frame_fraction);
  result.in_frame_fraction = in_frame_fraction;
  result.enough_in_frame =
      result.in_frame_fraction >= detector_params.min_in_frame_fraction;

  const auto [mae, confidence] =
      CompareMarker(warped, expected_marker, detector_params.warp_resolution);
  result.mean_abs_error = mae;
  result.confidence = confidence;

  result.detected = result.projected_in_front && result.enough_alignment &&
                    result.enough_area && result.enough_in_frame &&
                    (result.mean_abs_error <= detector_params.max_mean_abs_error);
  return result;
}

}  // namespace

ProjectedRArucoDetector::ProjectedRArucoDetector(
    const DroneCameraRenderParams& camera_params,
    const RArucoDetectorParams& detector_params)
    : camera_params_(camera_params),
      detector_params_(detector_params),
      X_BC_(RigidTransformd::Identity()),
      expected_marker_(UpsampleBinaryBits(
          BuildRecursiveMarkerBits(detector_params.marker.depth,
                                   detector_params.marker.inner_borders,
                                   detector_params.marker.external_border),
          detector_params.warp_resolution)) {
  const double pitch_rad = camera_params.pitch_down_deg * M_PI / 180.0;
  const Eigen::Vector3d look_direction(std::cos(pitch_rad), 0.0,
                                       -std::sin(pitch_rad));
  const Eigen::Vector3d down_reference(0.0, 0.0, -1.0);
  Eigen::Vector3d camera_y =
      down_reference - down_reference.dot(look_direction) * look_direction;
  if (camera_y.norm() < 1e-8) {
    camera_y = Eigen::Vector3d(0.0, -1.0, 0.0);
  }
  camera_y.normalize();
  const Eigen::Vector3d camera_z = look_direction.normalized();
  const Eigen::Vector3d camera_x = camera_y.cross(camera_z).normalized();
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix.col(0) = camera_x;
  rotation_matrix.col(1) = camera_y;
  rotation_matrix.col(2) = camera_z;
  X_BC_ = RigidTransformd(RotationMatrixd(rotation_matrix), camera_params.position);

  quadrotor_state_input_port_ = this->DeclareVectorInputPort(
                                    "quadrotor_state",
                                    drake::systems::BasicVector<double>(kQuadrotorStateSize))
                                    .get_index();
  moving_target_state_input_port_ = this->DeclareVectorInputPort(
                                        "moving_target_state",
                                        drake::systems::BasicVector<double>(kMovingTargetStateSize))
                                        .get_index();
  color_image_input_port_ = this->DeclareAbstractInputPort(
                                "color_image",
                                drake::Value<ImageRgba8U>(ImageRgba8U()))
                                .get_index();

  this->DeclareAbstractOutputPort("raruco_detection",
                                  &ProjectedRArucoDetector::CalcDetection);
  this->DeclareAbstractOutputPort("annotated_image",
                                  &ProjectedRArucoDetector::CalcAnnotatedImage);
}

void ProjectedRArucoDetector::CalcDetection(
    const drake::systems::Context<double>& context,
    lcmt_raruco_detection* output) const {
  const Eigen::VectorXd quadrotor_state =
      this->get_input_port(quadrotor_state_input_port_).Eval(context);
  const Eigen::VectorXd moving_target_state =
      this->get_input_port(moving_target_state_input_port_).Eval(context);
  const auto& image =
      this->get_input_port(color_image_input_port_).Eval<ImageRgba8U>(context);

  const DetectionComputation result =
      RunDetection(camera_params_, detector_params_, X_BC_, expected_marker_,
                   quadrotor_state, moving_target_state, image);

  output->utime = static_cast<int64_t>(std::llround(context.get_time() * 1e6));
  output->timestamp_millis = static_cast<int64_t>(std::llround(context.get_time() * 1e3));
  output->detected = result.detected ? 1 : 0;
  output->marker_id = detector_params_.marker.id;
  output->confidence = result.confidence;
  output->mean_abs_error = result.mean_abs_error;
  output->center_u = result.center_px.x();
  output->center_v = result.center_px.y();
  output->polygon_area_px = result.polygon_area_px;
  output->normal_alignment = result.normal_alignment;
  output->in_frame_fraction = result.in_frame_fraction;
  output->position_C[0] = result.position_C.x();
  output->position_C[1] = result.position_C.y();
  output->position_C[2] = result.position_C.z();
  for (int i = 0; i < 4; ++i) {
    output->corner_u[i] = result.corners_px[i].x();
    output->corner_v[i] = result.corners_px[i].y();
  }
}

void ProjectedRArucoDetector::CalcAnnotatedImage(
    const drake::systems::Context<double>& context,
    ImageRgba8U* output) const {
  const Eigen::VectorXd quadrotor_state =
      this->get_input_port(quadrotor_state_input_port_).Eval(context);
  const Eigen::VectorXd moving_target_state =
      this->get_input_port(moving_target_state_input_port_).Eval(context);
  const auto& image =
      this->get_input_port(color_image_input_port_).Eval<ImageRgba8U>(context);

  CopyImage(image, output);

  const DetectionComputation result =
      RunDetection(camera_params_, detector_params_, X_BC_, expected_marker_,
                   quadrotor_state, moving_target_state, image);

  const std::array<uint8_t, 4> color =
      result.detected ? std::array<uint8_t, 4>{40, 220, 40, 255}
                      : std::array<uint8_t, 4>{220, 50, 50, 255};
  const std::array<uint8_t, 4> center_color{255, 220, 0, 255};
  for (int i = 0; i < 4; ++i) {
    DrawLine(output, result.corners_px[i], result.corners_px[(i + 1) % 4], color);
  }
  DrawLine(output, result.corners_px[0], result.corners_px[2], color);
  DrawLine(output, result.corners_px[1], result.corners_px[3], color);
  DrawCross(output, result.center_px, 5, center_color);
}

}  // namespace systems
}  // namespace uav_delivery
