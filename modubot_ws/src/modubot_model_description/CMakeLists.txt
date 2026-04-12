cmake_minimum_required(VERSION 3.8)
project(modubot_model_description)

# 1. Encontrar dependências
find_package(ament_cmake REQUIRED)

# 2. Instalar as pastas de recursos do robô
# Adicionamos 'launch' na lista para que o comando ros2 launch funcione
install(
  DIRECTORY 
    urdf 
    meshes 
    launch
    scripts
  DESTINATION share/${PROJECT_NAME}
)

# 3. Opções de compilação (opcional, mantido do seu original)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 4. Testes (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 5. Fechamento do pacote
ament_package()