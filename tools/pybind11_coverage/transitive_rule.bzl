GlobFiles = provider("files")

def _file_globber_impl(ctx):
    all_files = []
    for f in ctx.attr.files:
        all_files = all_files + f.files.to_list()
    all_files_depset = depset(all_files)

    return [
        GlobFiles(files = all_files_depset),
        DefaultInfo(files = all_files_depset),
    ]

files_globber = rule(
    implementation = _file_globber_impl,
    attrs = {
        "files": attr.label_list(allow_files = True),
    }
)

def get_trans_deps(deps):
    depset_list = ([dep[GlobFiles].files for dep in deps])
    files_list = []
    for dep in depset_list:
        for f in dep.to_list():
            files_list.append(f)
    return depset(files_list)

def _main_globber_impl(ctx):
    foocc = ctx.executable._foocc
    out = ctx.outputs.out
    trans_srcs = get_trans_deps(ctx.attr.deps)
    srcs_list = trans_srcs.to_list()

    ctx.actions.run(
        executable = foocc,
        arguments = [out.path] + [src.path for src in srcs_list],
        inputs = srcs_list,
        tools = [foocc],
        outputs = [out],
    )

def add_for_pybind_coverage():
    files_globber(
            name = "cc_glob",
            files = native.glob(["*.cc"]),
            visibility = ["//bindings/pydrake:__pkg__"],
    )


main_globber = rule(
    implementation = _main_globber_impl,
    attrs = {
        "deps": attr.label_list(allow_files = True),
        "_foocc": attr.label(
            default = Label("//tools/pybind11_coverage:parse_pybind11_doc_xml"),
            allow_files = True,
            executable = True,
            cfg = "host",
        ),
    },
    outputs = {"out": "%{name}.out"},
)
