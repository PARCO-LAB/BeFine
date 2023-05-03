/**
 * @file camera.hpp
 * @date 3/12/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Default Camera used.
 */

#ifndef CAMERA_CAMERA_HPP
#define CAMERA_CAMERA_HPP

#include "maeve/camera/folder/folder.hpp"

#if defined(LIB_ALL) && LIB_ALL
#include "maeve/camera/zed/zed.hpp"
#include "maeve/camera/realsense/realsense.hpp"
#else // defined(LIB_ALL) && LIB_ALL

#if defined(ZEDCAMERA)
#include "maeve/camera/zed/zed.hpp"
#elif defined(REALSENSECAMERA)
#include "maeve/camera/realsense/realsense.hpp"
#else 
#error "LIB_CAMERA not recognized"
#endif

#endif // defined(LIB_ALL) && LIB_ALL

namespace maeve {

#if defined(LIB_ALL) && LIB_ALL
using Camera = Zed;
using CameraSetting = ZedSetting;
#else // defined(LIB_ALL) && LIB_ALL

#if defined(ZEDCAMERA)
using Camera = Zed;
using CameraSetting = ZedSetting;
#elif defined(REALSENSECAMERA)
using Camera = RealSense;
using CameraSetting = RealSenseSetting;
#else 
#error "LIB_CAMERA not recognized"
#endif

#endif // defined(LIB_ALL) && LIB_ALL



} // namespace maeve

#endif // CAMERA_CAMERA_HPP
