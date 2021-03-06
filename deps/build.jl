using BinaryProvider

const verbose = "--verbose" in ARGS
const prefix = Prefix(get([a for a in ARGS if a != "--verbose"], 1, joinpath(@__DIR__, "usr")))
products = [
    LibraryProduct(prefix, ["libPixhawkData"], :PixhawkData),
]

if Sys.iswindows()
    using Pkg
    Pkg.add(PackageSpec(name="Cxx", rev="gn-patch-crash"))
end


# TODO create a repo
bin_prefix = "http://www.github.com/akshayhiregoudar/PixhawkDataBuilder/releases/download/v1.0"


# TODO check version number
download_info = Dict(
    Linux(:x86_64, libc=:glibc, compiler_abi=CompilerABI(:gcc7)) => ("$bin_prefix/PixhawkData.v1.0.x86_64-linux-gnu-gcc7.tar.gz", "57f0d9f9e89f05db78ec41c6cb5878b8e2290e9868175f556a34bcfc61df0578"),
    #Linux(:x86_64, libc=:glibc, compiler_abi=CompilerABI(:gcc8)) => ("$bin_prefix/PixhawkData.v1.0.x86_64-linux-gnu-gcc8.tar.gz", "7caaa648e059c354892a5503b30bf3b2d368469b4295f54564f8a212fa3e2893"),
    #Windows(:x86_64, compiler_abi=CompilerABI(:gcc7)) => ("$bin_prefix/PixhawkData.v1.0.x86_64-w64-mingw32-gcc7.tar.gz", "782f52898e81c51abd8930c34184a6f99d78e4a49e7a6c753b9eb0741e8bcdfc"),
    #Windows(:x86_64, compiler_abi=CompilerABI(:gcc8)) => ("$bin_prefix/PixhawkData.v1.0.x86_64-w64-mingw32-gcc8.tar.gz", "283473593d120f968366f85da52bd4341ac94715c49476adf9bdd1688b445411"),
    #MacOS(:x86_64) => ("$bin_prefix/PixhawkData.v1.0.x86_64-apple-darwin14.tar.gz", "de9236acab1975a06f218d2939f6c319fb0bc67546e89cdc5c85ef90be509cb4"),
)

unsatisfied = any(!satisfied(p; verbose=verbose) for p in products)
dl_info = choose_download(download_info, platform_key_abi())
if dl_info === nothing && unsatisfied
    # If we don't have a compatible .tar.gz to download, complain.
    # Alternatively, you could attempt to install from a separate provider,
    # build from source or something even more ambitious here.
    error("Your platform (\"$(Sys.MACHINE)\", parsed as \"$(triplet(platform_key_abi()))\") is not supported by this package!")
end

# If we have a download, and we are unsatisfied (or the version we're
# trying to install is not itself installed) then load it up!
if unsatisfied || !isinstalled(dl_info...; prefix=prefix)
    # Download and install binaries
    install(dl_info...; prefix=prefix, force=true, verbose=verbose)
end

# Write out a deps.jl file that will contain mappings for our products
write_deps_file(joinpath(@__DIR__, "deps.jl"), products, verbose=verbose)
