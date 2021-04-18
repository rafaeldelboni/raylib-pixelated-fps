add_executable(
  example_app
  example_app.c
)

add_dependencies(
  example_app
  resources
)

add_executable(
  ode_app
  ode_app.c
)

add_dependencies(
  ode_app
  resources
)

target_link_libraries(
  example_app
  raylib
  ODE
)

target_link_libraries(
  ode_app
  raylib
  rlights
  ODE
)
