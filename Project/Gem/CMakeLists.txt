#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

# Query the gem name from the gem.json file if possible
# otherwise fallback to using ROSCon2023Demo
o3de_find_ancestor_gem_root(gem_path gem_name "${CMAKE_CURRENT_SOURCE_DIR}")
if (NOT gem_name)
    set(gem_name "ROSCon2023Demo")
endif()

# Fallback to using the current source CMakeLists.txt directory as the gem root path
if (NOT gem_path)
    set(gem_path ${CMAKE_CURRENT_SOURCE_DIR})
endif()

set(gem_json ${gem_path}/gem.json)

o3de_restricted_path(${gem_json} gem_restricted_path gem_parent_relative_path)

# Currently we are in the ROSCon2023Demo/Code folder: ${CMAKE_CURRENT_LIST_DIR}
# Get the platform specific folder ${pal_dir} for the current folder: ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME}
# Note: o3de_pal_dir will take care of the details for us, as this may be a restricted platform
#       in which case it will see if that platform is present here or in the restricted folder.
#       i.e. It could here : ROSCon2023Demo/Code/Platform/<platform_name>  or
#            <restricted_folder>/<platform_name>/ROSCon2023Demo/Code
o3de_pal_dir(pal_dir ${CMAKE_CURRENT_SOURCE_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

# Now that we have the platform abstraction layer (PAL) folder for this folder, thats where we will find the
# traits for this platform. Traits for a platform are defines for things like whether or not something in this project
# is supported by this platform.
include(${pal_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

# Now that we have loaded our project traits for this platform, see if this project is even supported on this platform.
# If its not supported we just return after including the unsupported.
if(NOT PAL_TRAIT_ROSCon2023Demo_SUPPORTED)
    return()
endif()

# We are on a supported platform, so add the ${gem_name} target
# Note: We include the common files and the platform specific files which are set in ROSCon2023Demo_files.cmake and
# in ${pal_dir}/ROSCon2023Demo_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake

# The ${gem_name}.Private.Object target is an internal target
# It should not be used outside of this CMakeLists.txt
ly_add_target(
    NAME ${gem_name}.Private.Object STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ROSCon2023Demo_files.cmake
        ${pal_dir}/ROSCon2023Demo_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            AZ::AzGameFramework
            Gem::Atom_AtomBridge.Static
            Gem::ROS2.Static
            Gem::RecastNavigation.API
            Gem::EMotionFX.Static
            Gem::LmbrCentral.API
)

target_depends_on_ros2_packages(${gem_name}.Private.Object moveit_msgs)

ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ROSCon2023Demo_shared_files.cmake
        ${pal_dir}/ROSCon2023Demo_shared_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::${gem_name}.Private.Object
            AZ::AzCore
            Gem::RecastNavigation.API
            Gem::ROS2.Static
)

target_depends_on_ros2_packages(${gem_name}.Private.Object  vision_msgs lane_provider_msgs)
target_depends_on_ros2_packages(${gem_name} vision_msgs lane_provider_msgs)

# if enabled, ${gem_name} is used by all kinds of applications
ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Tools    NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Clients  NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Servers  NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Unified  NAMESPACE Gem TARGETS Gem::${gem_name})

################################################################################
# Gem dependencies
################################################################################

# Query the project name from the nearest project.json file in a directory at or above
# the current one.
# This gem is the project gem and therefore should be part of the project that is using it
o3de_find_ancestor_project_root(project_path project_name "${CMAKE_CURRENT_SOURCE_DIR}")

# If the project name could not be queried from a project.json file, then fallback
# to using the name of the project provided when the project template was instantiated
if (NOT project_name)
    set(project_name ROSCon2023Demo)
endif()

# Enable the specified list of gems from GEM_FILE or GEMS list for this specific project:
ly_enable_gems(PROJECT_NAME ${project_name} GEM_FILE enabled_gems.cmake)
