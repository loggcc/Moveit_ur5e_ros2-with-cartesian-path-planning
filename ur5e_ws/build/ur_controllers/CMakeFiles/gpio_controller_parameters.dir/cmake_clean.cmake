file(REMOVE_RECURSE
  "include/gpio_controller_parameters.hpp"
  "include/ur_controllers/gpio_controller_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/gpio_controller_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
