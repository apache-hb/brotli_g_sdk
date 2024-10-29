project('brotli_g_sdk', 'cpp',
    version : '1.1.0',
    default_options : [
        'cpp_std=c++17',
        'default_library=static',
    ],
    license : 'LicenseRef-Proprietary',
    license_files : [ 'LICENSE.txt' ],
    meson_version : '>=1.1.0'
)

cpp = meson.get_compiler('cpp')

common = dependency('libbrotlicommon')
encode = dependency('libbrotlidec')
decode = dependency('libbrotlienc')

src = [
    'src/common/BrotligDataConditioner.cpp',
    'src/common/BrotligUtils.cpp',
    'src/common/BrotligCommand.cpp',
    'src/common/BrotligBitWriter.cpp',

    'src/decoder/BrotligHuffmanTable.cpp',
    'src/decoder/PageDecoder.cpp',

    'src/encoder/BrotligSwizzler.cpp',
    'src/encoder/BrotligHuffman.cpp',
    'src/encoder/PageEncoder.cpp',

    'src/BrotligDecoder.cpp',
    'src/BrotligEncoder.cpp',
    'src/DataStream.cpp',
]

inc = include_directories('inc', 'inc/common', 'inc/decoder', 'inc/encoder')

shader = files('src/decoder/BrotliGCompute.hlsl')

libbrotlig = library('brotli_g_sdk', src,
    include_directories : [ inc ],
    dependencies : [ common, encode, decode ],
    extra_files : shader,
    cpp_args : [ '-D_HAS_EXCEPTIONS=0' ],
    override_options : [ 'unity=off', 'cpp_eh=none' ]
)

brotlig_dep = declare_dependency(
    link_with : libbrotlig,
    include_directories : inc,
    dependencies : [ common, encode, decode ]
)

build_sample = get_option('build_sample').disable_auto_if(meson.is_subproject())
if build_sample.allowed()
    fs = import('fs')
    d3dx12 = dependency('directx-headers')
    dxcompiler = dependency('dxcompiler')
    d3d12 = cpp.find_library('d3d12')
    dxgi = cpp.find_library('dxgi')
    dxguid = cpp.find_library('dxguid')

    executable('brotli_g_sample', 'sample/brotlig_cli.cpp', 'sample/BrotligGPUDecoder.cpp', fs.copyfile(shader),
        dependencies : [ brotlig_dep, d3dx12, dxcompiler, d3d12, dxgi, dxguid ],
        cpp_args : [ '-DUSE_BROTLI_CODEC', '-DUSE_GPU_DECOMPRESSION' ],
        override_options : [ 'unity=off', 'cpp_eh=default' ]
    )
endif

meson.override_dependency('brotli_g_sdk', brotlig_dep)

install_data(meson.project_license_files(),
    install_dir : get_option('datadir') / 'licenses',
    rename : meson.project_name() + '.LICENSE.txt'
)
