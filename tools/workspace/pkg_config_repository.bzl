load("@drake//tools/workspace:pkg_config.bzl", "setup_pkg_config_repository")

def _system_pkg_config_repository_impl(repository_ctx):
    result = setup_pkg_config_repository(repository_ctx)
    if result.error != None:
        fail("Unable to configure @{}: {}".format(
            repository_ctx.name,
            result.error,
        ))

system_pkg_config_repository = repository_rule(
    implementation = _system_pkg_config_repository_impl,
    attrs = {
        "modname": attr.string(mandatory = True),
    },
    configure = True,
    local = True,
)
