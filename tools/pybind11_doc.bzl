# -*- python -*-

def _impl(ctx):
    output = ctx.outputs.output_filename
    mkdoc_label = ctx.files.mkdoc_label
    inputs = ctx.files.inputs
    includes = []
    system_includes = []
    quote_includes = []
    transitive_headers = []

    for f in ctx.attr.deps:
        #print(f.files)

        if hasattr(f, "cc"):
            includes += f.cc.include_directories
            system_includes += f.cc.system_include_directories
            quote_includes += f.cc.quote_include_directories
            transitive_headers += f.cc.transitive_headers.to_list()

    all_includes = includes + system_includes + quote_includes
    include_string = "-Ibazel-drake-distro/external/eigen "

    input_string = " ".join([i.path for i in inputs])
    for p in transitive_headers:
        ext = p.extension
        # print(dir(p))
        if ext == "h":
            p_short_path = p.short_path
            base_path = "tools/install/libdrake/_virtual_includes/drake_shared_library/drake/"

            if p_short_path.startswith(base_path):
                after_replace = p_short_path.replace(base_path, "")
                input_string = input_string + " " + after_replace

    print(input_string)

    for inc_path in all_includes:
        include_string += "-I{} ".format(inc_path)

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
        "output_filename": attr.output(mandatory = True),
        "inputs": attr.label_list(allow_files = True),
    },
    implementation = _impl,
)
