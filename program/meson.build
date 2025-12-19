project(
  'program',
  'c',
  meson_version : '>= 1.3.0',
  version : '0.1',
  default_options : [
    'warning_level=3',
    'buildtype=debug'
  ],
)

incdir = include_directories('include')

gtk_dep = dependency('gtk4', required: true)
threads_dep = dependency('threads')
dependencies = [
    gtk_dep,
    threads_dep
]

sources = [
  'src/program.c',
  'src/serial.c',
  'src/tty.c',
  'src/interface.c',
  'src/logger.c',
  'src/buffer.c'
]

exe = executable(
  'program',
  sources,
  include_directories : incdir,
  dependencies : dependencies,
  install : true,
)

