add_executable(
  example_app
  example_app.c
)

add_dependencies(
  example_app
  resources
)

target_link_libraries(
  example_app
  raylib
)
