# -*- python -*-

def _impl(ctx):
    output = ctx.outputs.output_filename
    mkdoc_label = ctx.files.mkdoc_label
    inputs = ctx.files.inputs
    includes = []
    system_includes = []
    quote_includes = []

    for f in ctx.attr.deps:
        if hasattr(f, "cc"):
            includes += f.cc.include_directories
            system_includes += f.cc.system_include_directories
            quote_includes += f.cc.quote_include_directories

    all_includes = includes + system_includes + quote_includes
    include_string = "-Ibazel-drake-distro/external/eigen  "

    for inc_path in all_includes:
        include_string += "-I{} ".format(inc_path)

    input_string = " ".join([i.path for i in inputs])

    ctx.actions.run_shell(
        outputs = [output],
        inputs = inputs + mkdoc_label,
        command = "LD_LIBRARY_PATH=/usr/lib/llvm-4.0/lib {} \
            -std=c++14 {} {} > {}".format(mkdoc_label[0].path,
            include_string, input_string, output.path),
    )


generate_pybind11_doc = rule(
    attrs = {
        "deps": attr.label_list(),
        "mkdoc_label": attr.label(mandatory = True, allow_files = True),
        "output_filename": attr.string(mandatory = True),
        "inputs": attr.label_list(allow_files = True),
    },
    implementation = _impl,
    outputs = {"output_filename": "%{output_filename}"}
)
