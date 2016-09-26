/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __QCAMERAFOVCONTROL_H__
#define __QCAMERAFOVCONTROL_H__

#include <utils/Mutex.h>

#include "cam_intf.h"

typedef enum {
    CAM_TYPE_WIDE,
    CAM_TYPE_TELE,
    CAM_COUNT
} cam_type;

typedef enum {
    CAM_POSITION_LEFT,
    CAM_POSITION_RIGHT
} cam_relative_position;

typedef enum {
    READY,
    NOT_READY
} status;

typedef enum {
    AE_SETTLED,
    AE_CONVERGING
} ae_status;

typedef enum {
    AF_VALID,
    AF_INVALID
} af_status;

typedef enum {
    AWB_SETTLED,
    AWB_CONVERGING
} awb_status;

typedef struct {
    ae_status   statusAE;
    af_status   statusAF;
    awb_status  statusAWB;
} status_3A_t;

typedef struct {
    status_3A_t camMain;
    status_3A_t camAux;
} dual_cam_3A_status_t;

typedef struct {
    status   status;
    uint32_t shiftHorz;
    uint32_t shiftVert;
} spatial_align_metadata_t;

typedef enum {
    STATE_WIDE,
    STATE_TRANSITION_WIDE_TO_TELE,
    STATE_TELE,
    STATE_TRANSITION_TELE_TO_WIDE
} dual_cam_state;

typedef struct {
    float    cropRatio;
    float    cutOverFactor;
    float    cutOverMainToAux;
    float    cutOverAuxToMain;
    float    transitionHigh;
    float    transitionLow;
    uint32_t waitTimeForHandoffMs;
} dual_cam_transition_params_t;

typedef struct {
    uint32_t                     zoomMain;
    uint32_t                     zoomAux;
    cam_sync_type_t              camWide;
    cam_sync_type_t              camTele;
    uint32_t                     shiftHorzAdjMain;
    dual_cam_state               camState;
    dual_cam_3A_status_t         status3A;
    dual_cam_transition_params_t transitionParams;
    cam_dimension_t              previewSize;
    spatial_align_metadata_t     spatialAlign;
} fov_control_data_t;

typedef struct {
    float    percentMarginHysterisis;
    float    percentMarginAux;
    float    percentMarginMain;
    uint32_t waitTimeForHandoffMs;
} fov_control_config_t;

typedef struct{
    uint32_t sensorStreamWidth;
    uint32_t sensorStreamHeight;
    float    focalLengthMm;
    float    pixelPitchUm;
} intrinsic_cam_params_t;

typedef struct {
    uint32_t               minFocusDistanceCm;
    float                  baselineMm;
    float                  rollDegrees;
    float                  pitchDegrees;
    float                  yawDegrees;
    cam_relative_position  positionAux;
    intrinsic_cam_params_t paramsMain;
    intrinsic_cam_params_t paramsAux;
} dual_cam_params_t;

typedef struct {
    cam_sync_type_t camMasterPreview;
    cam_sync_type_t camMaster3A;
    uint32_t        camState;
    bool            snapshotFusion;
} fov_control_result_t;


#define ZOOM_TABLE_SIZE 182
// TODO : Replace zoom table with the zoom ratio table from capabilities.
// That zoom ratio table has ratios normalized to 100.
static const uint32_t zoomTableDualCam[ZOOM_TABLE_SIZE] = {
                              4096, 4191, 4289, 4389, 4492,
                              4597, 4705, 4815, 4927, 5042,
                              5160, 5281, 5404, 5531, 5660,
                              5792, 5928, 6066, 6208, 6353,
                              6501, 6653, 6809, 6968, 7131,
                              7298, 7468, 7643, 7822, 8004,
                              8192, 8383, 8579, 8779, 8985,
                              9195, 9410, 9630, 9855, 10085,
                              10321, 10562, 10809, 11062, 11320,
                              11585, 11856, 12133, 12416, 12706,
                              13003, 13307, 13619, 13937, 14263,
                              14596, 14937, 15286, 15644, 16009,
                              16384, 16766, 17158, 17559, 17970,
                              18390, 18820, 19260, 19710, 20171,
                              20642, 21125, 21618, 22124, 22641,
                              23170, 23712, 24266, 24833, 25413,
                              26007, 26615, 27238, 27874, 28526,
                              29192, 29875, 30573, 31288, 32019,
                              32768, 33533, 34317, 35119, 35940,
                              36780, 37640, 38520, 39420, 40342,
                              41285, 42250, 43237, 44248, 45282,
                              46340, 47424, 48532, 49666, 50827,
                              52015, 53231, 54476, 55749, 57052,
                              58385, 59750, 61147, 62576, 64039,
                              65536, 67067, 68635, 70239, 71881,
                              73561, 75281, 77040, 78841, 80684,
                              82570, 84500, 86475, 88496, 90565,
                              92681, 94848, 97065, 99334, 101655,
                              104031, 106463, 108952, 111498, 114104,
                              116771, 119501, 122294, 125152, 128078,
                              131072, 134135, 137270, 140479, 143763,
                              147123, 150562, 154081, 157682, 161368,
                              165140, 169000, 172950, 176993, 181130,
                              185363, 189696, 194130, 198668, 203311,
                              208063, 212927, 217904, 222997, 228209,
                              233543, 239002, 244589, 250305, 256156,
                              262144, 999999
};


using namespace android;

namespace qcamera {

class QCameraFOVControl {
public:
    ~QCameraFOVControl();

    static QCameraFOVControl* create(cam_capability_t *capsMainCam,
                                     cam_capability_t* capsAuxCam);

    cam_capability_t consolidateCapabilities(cam_capability_t* capsMainCam,
                                             cam_capability_t* capsAuxCam);
    int32_t translateInputParams(parm_buffer_t* paramsMainCam,
                                 parm_buffer_t *paramsAuxCam);
    metadata_buffer_t* processResultMetadata(metadata_buffer_t* metaMainCam,
                                             metadata_buffer_t* metaAuxCam);
    fov_control_result_t getFovControlResult();

private:
    QCameraFOVControl();
    float calculateBasicFovRatio();
    float calculateFovAdjustmentWorstCaseDisparity();
    float calculateFovAdjustmentRollPitchYaw();
    float combineFovAdjustment(float fovAdjustBasic,
                               float fovAdjustFromDisparity,
                               float fovAdjustFromRollPitchYaw);
    void  calculateDualCamTransitionParams(float fovAdjustBasic,
                                           float zoomTranslationFactor);


    void convertUserZoomToMainAndAux(uint32_t zoom);
    uint32_t readjustZoomForAux(uint32_t zoomMain);
    uint32_t readjustZoomForMain(uint32_t zoomAux);
    uint32_t findZoomRatio(uint32_t zoom);
    inline uint32_t findZoomValue(uint32_t zoomRatio);

    cam_face_detection_data_t translateRoiFD(cam_face_detection_data_t faceDetectionInfo);
    cam_roi_info_t translateFocusAreas(cam_roi_info_t roiAfMain);
    cam_set_aec_roi_t translateMeteringAreas(cam_set_aec_roi_t roiAecMain);
    void convertDisparityForInputParams();

    Mutex                           mMutex;
    fov_control_config_t            mFovControlConfig;
    fov_control_data_t              mFovControlData;
    fov_control_result_t            mFovControlResult;
    dual_cam_params_t               mDualCamParams;
};

}; // namespace qcamera

#endif /* __QCAMERAFOVCONTROL_H__ */
