
cmake_minimum_required(VERSION 3.16)

project(P4_VehicleSystemIdentification
    VERSION 0.1.0
    LANGUAGES CXX
)

# ============================================================
# 1. C++ Configuration
# ============================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# 2. Project Paths
# ============================================================

set(P4_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(P4_IO_PRIVATE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io
)

set(P4_PARSERS_PRIVATE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io/parsers
)

set(P4_DATA_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/data/old_vito_not_used
)

# ============================================================
# 3. Libraries
# ============================================================

# ------------------------------------------------------------
# 3.1 Vehicle Dynamics Models
# ------------------------------------------------------------

add_library(p4_models STATIC
    src/models/FirstOrderYawModel.cpp
    src/models/SteeringActuatorModel.cpp
    src/models/LinearBicycleModel.cpp
)

target_include_directories(p4_models
    PUBLIC
        ${P4_INCLUDE_DIR}
)

target_compile_features(p4_models
    PUBLIC cxx_std_17
)

add_library(p4::models ALIAS p4_models)

# ------------------------------------------------------------
# 3.2 VBO Input / Output
# ------------------------------------------------------------

add_library(p4_io STATIC
    src/io/ControllerModel.cpp
    src/io/VBOData.cpp
    src/io/VBOReader.cpp
    src/io/IONIQ5CsvReader.cpp
    src/io/IONIQ5LoadSamples.cpp
    src/io/parsers/SectionedVBOParser.cpp
    src/io/parsers/TabularVBOParser.cpp
)

target_include_directories(p4_io
    PUBLIC
        ${P4_INCLUDE_DIR}

    PRIVATE
        ${P4_IO_PRIVATE_DIR}
        ${P4_PARSERS_PRIVATE_DIR}
)

target_compile_features(p4_io
    PUBLIC cxx_std_17
)

add_library(p4::io ALIAS p4_io)

add_library(p4_golf_data STATIC src/identification/GolfDataset.cpp)
target_include_directories(p4_golf_data PUBLIC ${P4_INCLUDE_DIR})
target_link_libraries(p4_golf_data PUBLIC p4::io)
target_compile_features(p4_golf_data PUBLIC cxx_std_17)
add_library(p4::golf_data ALIAS p4_golf_data)

add_library(p4_controller STATIC src/controller/ControllerRuntime.cpp)
target_include_directories(p4_controller PUBLIC ${P4_INCLUDE_DIR})
target_link_libraries(p4_controller PUBLIC p4::io p4::models)
target_compile_features(p4_controller PUBLIC cxx_std_17)
add_library(p4::controller ALIAS p4_controller)

# ------------------------------------------------------------
# 3.3 Maneuver Analysis
# ------------------------------------------------------------

add_library(p4_maneuvers STATIC
    src/maneuvers/ManeuverDetector.cpp
    src/maneuvers/ManeuverSegmenter.cpp
    src/maneuvers/ManeuverGrouper.cpp
    src/maneuvers/MajorExcursionDetector.cpp
    src/maneuvers/ManeuverPhaseAnalyzer.cpp
)

target_include_directories(p4_maneuvers
    PUBLIC
        ${P4_INCLUDE_DIR}
)

target_link_libraries(p4_maneuvers
    PUBLIC
        p4::io
)

target_compile_features(p4_maneuvers
    PUBLIC cxx_std_17
)

add_library(p4::maneuvers ALIAS p4_maneuvers)

# ============================================================
# 4. Applications
# ============================================================

# ------------------------------------------------------------
# 4.1 VBO Data Analysis
# ------------------------------------------------------------

add_executable(vbo_data_analysis
    apps/vbo_data_analysis.cpp
)

target_link_libraries(vbo_data_analysis
    PRIVATE p4::io
)

# ------------------------------------------------------------
# 4.2 Step Steer Candidate Analysis
# ------------------------------------------------------------

add_executable(maneuver_analysis
    apps/maneuver_analysis.cpp
)

target_link_libraries(maneuver_analysis
    PRIVATE p4::maneuvers
)

# ------------------------------------------------------------
# 4.3 Maneuver CSV Export
# ------------------------------------------------------------

add_executable(maneuver_export
    apps/maneuver_export.cpp
)

target_link_libraries(maneuver_export
    PRIVATE p4::maneuvers
)

# ------------------------------------------------------------
# 4.4 Maneuver Segmentation and Grouping
# ------------------------------------------------------------

add_executable(maneuver_segment_analysis
    apps/maneuver_segment_analysis.cpp
)

target_link_libraries(maneuver_segment_analysis
    PRIVATE p4::maneuvers
)

add_executable(major_excursion_analysis
    apps/major_excursion_analysis.cpp
)

target_link_libraries(major_excursion_analysis
    PRIVATE p4::maneuvers
)

add_executable(maneuver_phase_analysis
    apps/maneuver_phase_analysis.cpp
)

target_link_libraries(maneuver_phase_analysis
    PRIVATE p4::maneuvers
)

add_executable(identification_window_analysis
    apps/identification_window_analysis.cpp
)

target_link_libraries(identification_window_analysis
    PRIVATE p4::maneuvers
)

add_executable(model_a_identify_vbo
    apps/model_a_identify_vbo.cpp
)
target_link_libraries(model_a_identify_vbo PRIVATE p4::io p4::models)

# IONIQ 5 one-file smoke test and file-group experiments.
add_executable(model_a_identify_csv apps/model_a_identify_csv.cpp)
target_link_libraries(model_a_identify_csv PRIVATE p4::io p4::models)

add_executable(model_a_grouped_csv apps/model_a_grouped_csv.cpp)
target_link_libraries(model_a_grouped_csv PRIVATE p4::io p4::models)

add_executable(model_s_load_csv apps/model_s_load_csv.cpp)
target_link_libraries(model_s_load_csv PRIVATE p4::io)

add_executable(model_a_speed_cv apps/model_a_speed_cv.cpp)
target_link_libraries(model_a_speed_cv PRIVATE p4::golf_data)

add_executable(model_s_load_friction_cv apps/model_s_load_friction_cv.cpp)
target_link_libraries(model_s_load_friction_cv PRIVATE p4::io)

add_executable(model_b_ioniq5_cv apps/model_b_ioniq5_cv.cpp)
add_executable(model_b_golf_cv apps/model_b_golf_cv.cpp)
target_link_libraries(model_b_golf_cv PRIVATE p4::golf_data p4::models)
add_executable(controller_export_parity apps/controller_export_parity.cpp)
target_link_libraries(controller_export_parity PRIVATE p4::controller)
add_executable(controller_integration apps/controller_integration.cpp)
target_link_libraries(controller_integration PRIVATE p4::controller)
add_executable(golf_prediction_horizon apps/golf_prediction_horizon.cpp)
target_link_libraries(golf_prediction_horizon PRIVATE p4::golf_data p4::controller)
target_link_libraries(model_b_ioniq5_cv PRIVATE p4::io p4::models)

add_executable(dataset_manifest
    apps/dataset_manifest.cpp
)

target_link_libraries(dataset_manifest
    PRIVATE p4::maneuvers
)

add_executable(model_a_train
    apps/model_a_train.cpp
)

target_link_libraries(model_a_train
    PRIVATE p4::io p4::models
)

target_compile_features(model_a_train
    PRIVATE cxx_std_17
)

add_executable(model_a_evaluate
    apps/model_a_evaluate.cpp
)

target_link_libraries(model_a_evaluate
    PRIVATE p4::io p4::models
)

target_compile_features(model_a_evaluate
    PRIVATE cxx_std_17
)

# ============================================================
# 5. Testing
# ============================================================

include(CTest)

if(BUILD_TESTING)

    add_test(NAME controller_export_parity_fixture
        COMMAND controller_export_parity
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/golf_controller_model.json
            ${CMAKE_CURRENT_SOURCE_DIR}/config/golf_baselines.json
            ${CMAKE_CURRENT_BINARY_DIR}/controller_export_parity_fixture.json)

    add_executable(ioniq5_csv_reader_test tests/ioniq5_csv_reader_test.cpp)
    target_link_libraries(ioniq5_csv_reader_test PRIVATE p4::io)
    add_test(NAME ioniq5_csv_reader_test COMMAND ioniq5_csv_reader_test)

    # Model A
    add_executable(model_a_test
        tests/model_a_test.cpp
    )

    target_link_libraries(model_a_test
        PRIVATE p4::models
    )

    add_test(
        NAME model_a_step_response
        COMMAND model_a_test
    )

    # Model S
    add_executable(model_s_test
        tests/model_s_test.cpp
    )

    target_link_libraries(model_s_test
        PRIVATE p4::models
    )

    add_test(
        NAME model_s_step_response
        COMMAND model_s_test
    )

    # Model S Boundary
    add_executable(model_s_boundary_test
        tests/model_s_boundary_test.cpp
    )

    target_link_libraries(model_s_boundary_test
        PRIVATE p4::models
    )

    add_test(
        NAME model_s_boundary_test
        COMMAND model_s_boundary_test
    )

    add_executable(model_b_test tests/model_b_test.cpp)
    target_link_libraries(model_b_test PRIVATE p4::models)
    add_test(NAME model_b_2dof_test COMMAND model_b_test)

    # VBOData
    add_executable(vbo_data_test
        tests/vbo_data_test.cpp
    )

    target_link_libraries(vbo_data_test
        PRIVATE p4::io
    )

    add_test(
        NAME vbo_data_test
        COMMAND vbo_data_test
    )

    # VBO Parsing Utilities
    add_executable(vbo_parsing_utils_test
        tests/vbo_parsing_utils_test.cpp
    )

    target_include_directories(vbo_parsing_utils_test
        PRIVATE
            ${P4_IO_PRIVATE_DIR}
    )

    add_test(
        NAME vbo_parsing_utils_test
        COMMAND vbo_parsing_utils_test
    )

    # Sectioned VBO Parser
    add_executable(sectioned_vbo_parser_test
        tests/sectioned_vbo_parser_test.cpp
    )

    target_link_libraries(sectioned_vbo_parser_test
        PRIVATE p4::io
    )

    target_include_directories(sectioned_vbo_parser_test
        PRIVATE
            ${P4_PARSERS_PRIVATE_DIR}
    )


    # Tabular VBO Parser
    add_executable(tabular_vbo_parser_test
        tests/tabular_vbo_parser_test.cpp
    )

    target_link_libraries(tabular_vbo_parser_test
        PRIVATE p4::io
    )

    target_include_directories(tabular_vbo_parser_test
        PRIVATE
            ${P4_PARSERS_PRIVATE_DIR}
    )


    # Integrated VBO Reader
    add_executable(vbo_reader_integration_test
        tests/vbo_reader_integration_test.cpp
    )

    target_link_libraries(vbo_reader_integration_test
        PRIVATE p4::io
    )


    # Legacy reader checks were present but not registered with CTest.
    add_executable(vbo_reader_test tests/vbo_reader_test.cpp)
    target_link_libraries(vbo_reader_test PRIVATE p4::io)

    add_executable(vbo_reader_002_test tests/vbo_reader_002_test.cpp)
    target_link_libraries(vbo_reader_002_test PRIVATE p4::io)
    # The five real-log VBO checks require optional local recordings.
    if(EXISTS "${P4_DATA_DIR}/vblog_001.vbo" AND
       EXISTS "${P4_DATA_DIR}/vblog_002.vbo")
        add_test(NAME sectioned_vbo_parser_test
            COMMAND sectioned_vbo_parser_test ${P4_DATA_DIR}/vblog_001.vbo)
        add_test(NAME tabular_vbo_parser_test
            COMMAND tabular_vbo_parser_test ${P4_DATA_DIR}/vblog_002.vbo)
        add_test(NAME vbo_reader_integration_test
            COMMAND vbo_reader_integration_test
                ${P4_DATA_DIR}/vblog_001.vbo ${P4_DATA_DIR}/vblog_002.vbo)
        add_test(NAME vbo_reader_001_test
            COMMAND vbo_reader_test ${P4_DATA_DIR}/vblog_001.vbo)
        add_test(NAME vbo_reader_002_test
            COMMAND vbo_reader_002_test ${P4_DATA_DIR}/vblog_002.vbo)
    endif()

    # Step Steer Detector
    add_executable(maneuver_detector_test
        tests/maneuver_detector_test.cpp
    )

    target_link_libraries(maneuver_detector_test
        PRIVATE p4::maneuvers
    )

    add_test(
        NAME maneuver_detector_test
        COMMAND maneuver_detector_test
    )

endif()
