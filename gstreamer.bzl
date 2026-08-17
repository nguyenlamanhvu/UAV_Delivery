def _gstreamer_impl(repository_ctx):
    repository_ctx.symlink("/usr", "usr_dir")
    repository_ctx.file("BUILD.bazel", """
load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "gstreamer",
    hdrs = glob([
        "usr_dir/include/gstreamer-1.0/**/*.h",
        "usr_dir/include/glib-2.0/**/*.h",
        "usr_dir/lib/x86_64-linux-gnu/glib-2.0/include/**/*.h",
    ]),
    includes = [
        "usr_dir/include/gstreamer-1.0",
        "usr_dir/include/glib-2.0",
        "usr_dir/lib/x86_64-linux-gnu/glib-2.0/include",
    ],
    linkopts = [
        "-lgstrtspserver-1.0",
        "-lgstreamer-1.0",
        "-lgobject-2.0",
        "-lglib-2.0",
        "-lgstrtp-1.0",
        "-lgstapp-1.0",
        "-pthread",
    ],
    visibility = ["//visibility:public"],
)
""")

gstreamer_repository = repository_rule(
    implementation = _gstreamer_impl,
    local = True,
)

def _gstreamer_extension_impl(module_ctx):
    gstreamer_repository(name = "gstreamer")

gstreamer_extension = module_extension(
    implementation = _gstreamer_extension_impl,
)
