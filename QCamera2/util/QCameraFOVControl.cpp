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

#define LOG_TAG "QCameraFOVControl"

#include <utils/Errors.h>
#include "QCameraFOVControl.h"

extern "C" {
#include "mm_camera_dbg.h"
}

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraFOVControl constructor
 *
 * DESCRIPTION: class constructor
 *
 * PARAMETERS : none
 *
 * RETURN     : void
 *
 *==========================================================================*/
QCameraFOVControl::QCameraFOVControl()
{
    memset(&mDualCamParams,    0, sizeof(dual_cam_params_t));
    memset(&mFovControlConfig, 0, sizeof(fov_control_config_t));
    memset(&mFovControlData,   0, sizeof(fov_control_data_t));
    memset(&mFovControlResult, 0, sizeof(fov_control_result_t));

    mFovControlData.spatialAlign.status       = NOT_READY;
    mFovControlData.status3A.camMain.statusAF = AF_INVALID;
    mFovControlData.status3A.camAux.statusAF  = AF_INVALID;

    mFovControlResult.camMasterPreview  = CAM_TYPE_MAIN;
    mFovControlResult.camMaster3A       = CAM_TYPE_MAIN;
    mFovControlResult.camState          = (uint32_t)CAM_TYPE_MAIN;
    mFovControlResult.snapshotFusion    = false;
}


/*===========================================================================
 * FUNCTION   : QCameraFOVControl destructor
 *
 * DESCRIPTION: class destructor
 *
 * PARAMETERS : none
 *
 * RETURN     : void
 *
 *==========================================================================*/
QCameraFOVControl::~QCameraFOVControl()
{
}


/*===========================================================================
 * FUNCTION   : create
 *
 * DESCRIPTION: This is a static method to create FOV-control object. It calls
 *              private constructor of the class and only returns a valid object
 *              if it can successfully initialize the FOV-control.
 *
 * PARAMETERS :
 *  @capsMain : The capabilities for the main camera
 *  @capsAux  : The capabilities for the aux camera
 *
 * RETURN     : Valid object pointer if succeeds
 *              NULL if fails
 *
 *==========================================================================*/
QCameraFOVControl* QCameraFOVControl::create(cam_capability_t *capsMainCam,
                                             cam_capability_t* capsAuxCam)
{
    QCameraFOVControl *pFovControl  = NULL;

    if (capsMainCam && capsAuxCam) {
        // Create FOV control object
        pFovControl = new QCameraFOVControl();

        if (pFovControl) {
            float fovAdjustBasic;
            float fovAdjustFromDisparity;
            float fovAdjustFromRollPitchYaw;
            float zoomTranslationFactor;

            // TODO : Replace the hardcoded values for mFovControlConfig and mDualCamParams below
            // with the ones extracted from capabilities when available in eeprom.
            pFovControl->mFovControlConfig.percentMarginHysterisis  = 5;
            pFovControl->mFovControlConfig.percentMarginMain        = 10;
            pFovControl->mFovControlConfig.percentMarginAux         = 15;
            pFovControl->mFovControlConfig.waitTimeForHandoffMs     = 1000;

            pFovControl->mDualCamParams.paramsMain.sensorStreamWidth  = 4208;
            pFovControl->mDualCamParams.paramsMain.sensorStreamHeight = 3120;
            pFovControl->mDualCamParams.paramsMain.pixelPitchUm       = 1.12;
            pFovControl->mDualCamParams.paramsMain.focalLengthMm      = 3.5;
            pFovControl->mDualCamParams.paramsAux.sensorStreamWidth   = 4208;
            pFovControl->mDualCamParams.paramsAux.sensorStreamHeight  = 3120;
            pFovControl->mDualCamParams.paramsAux.pixelPitchUm        = 1.12;
            pFovControl->mDualCamParams.paramsAux.focalLengthMm       = 7;
            pFovControl->mDualCamParams.baselineMm                    = 9.5;
            pFovControl->mDualCamParams.minFocusDistanceCm            = 30;
            pFovControl->mDualCamParams.rollDegrees                   = 1.0;
            pFovControl->mDualCamParams.pitchDegrees                  = 1.0;
            pFovControl->mDualCamParams.yawDegrees                    = 1.0;
            pFovControl->mDualCamParams.positionAux                   = CAM_POSITION_LEFT;

            fovAdjustBasic            = pFovControl->calculateBasicFovRatio();
            fovAdjustFromDisparity    = pFovControl->calculateFovAdjustmentWorstCaseDisparity();
            fovAdjustFromRollPitchYaw = pFovControl->calculateFovAdjustmentRollPitchYaw();

            if (fovAdjustBasic            >= 1.0 &&
                fovAdjustFromDisparity    >= 1.0 &&
                fovAdjustFromRollPitchYaw >= 1.0) {
                zoomTranslationFactor = pFovControl->combineFovAdjustment(fovAdjustBasic,
                                            fovAdjustFromDisparity, fovAdjustFromRollPitchYaw);

                pFovControl->calculateDualCamTransitionParams(fovAdjustBasic,
                                                                zoomTranslationFactor);

                if (pFovControl->mDualCamParams.paramsMain.focalLengthMm <
                    pFovControl->mDualCamParams.paramsAux.focalLengthMm) {
                    pFovControl->mFovControlData.camWide  = CAM_TYPE_MAIN;
                    pFovControl->mFovControlData.camTele  = CAM_TYPE_AUX;
                    pFovControl->mFovControlData.camState = STATE_WIDE;
                } else {
                    pFovControl->mFovControlData.camWide  = CAM_TYPE_AUX;
                    pFovControl->mFovControlData.camTele  = CAM_TYPE_MAIN;
                    pFovControl->mFovControlData.camState = STATE_TELE;
                }

                pFovControl->mFovControlData.status3A.camAux.statusAF  = AF_VALID;
                pFovControl->mFovControlData.status3A.camMain.statusAF = AF_VALID;
            } else {
                LOGE("FOV-control: Failed to create an object");
                delete pFovControl;
                pFovControl = NULL;
            }
        } else {
            LOGE("FOV-control: Failed to allocate memory for FOV-control object");
        }
    }

    return pFovControl;
}


/*===========================================================================
 * FUNCTION    : consolidateCapabilities
 *
 * DESCRIPTION : Combine the capabilities from main and aux cameras to return
 *              the consolidated capabilities.
 *
 * PARAMETERS  :
 * @capsMainCam: Capabilities for the main camera
 * @capsAuxCam : Capabilities for the aux camera
 *
 * RETURN      : Consolidated capabilities
 *
 *==========================================================================*/
cam_capability_t QCameraFOVControl::consolidateCapabilities(
                                        cam_capability_t *capsMainCam,
                                        cam_capability_t *capsAuxCam)
{
    cam_capability_t capsConsolidated;
    memcpy(&capsConsolidated, capsMainCam, sizeof(cam_capability_t));
    if ((capsMainCam != NULL) &&
        (capsAuxCam  != NULL)) {
        // Consolidate preview sizes
        uint32_t previewSizesTblCntMain  = capsMainCam->preview_sizes_tbl_cnt;
        uint32_t previewSizesTblCntAux   = capsAuxCam->preview_sizes_tbl_cnt;
        uint32_t previewSizesTblCntFinal = 0;

        for (uint32_t i = 0; i < previewSizesTblCntMain; ++i) {
            for (uint32_t j = 0; j < previewSizesTblCntAux; ++j) {
                if ((capsMainCam->preview_sizes_tbl[i].width ==
                     capsAuxCam->preview_sizes_tbl[j].width) &&
                    (capsMainCam->preview_sizes_tbl[i].height ==
                     capsAuxCam->preview_sizes_tbl[j].height)) {
                    if (previewSizesTblCntFinal != i) {
                        capsConsolidated.preview_sizes_tbl[previewSizesTblCntFinal].width =
                           capsAuxCam->preview_sizes_tbl[j].width;
                        capsConsolidated.preview_sizes_tbl[previewSizesTblCntFinal].height =
                           capsMainCam->preview_sizes_tbl[j].height;
                    }
                    ++previewSizesTblCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.preview_sizes_tbl_cnt = previewSizesTblCntFinal;

        // Consolidate video sizes
        uint32_t videoSizesTblCntMain  = capsMainCam->video_sizes_tbl_cnt;
        uint32_t videoSizesTblCntAux   = capsAuxCam->video_sizes_tbl_cnt;
        uint32_t videoSizesTblCntFinal = 0;

        for (uint32_t i = 0; i < videoSizesTblCntMain; ++i) {
            for (uint32_t j = 0; j < videoSizesTblCntAux; ++j) {
                if ((capsMainCam->video_sizes_tbl[i].width ==
                     capsAuxCam->video_sizes_tbl[j].width) &&
                    (capsMainCam->video_sizes_tbl[i].height ==
                     capsAuxCam->video_sizes_tbl[j].height)) {
                    if (videoSizesTblCntFinal != i) {
                        capsConsolidated.video_sizes_tbl[videoSizesTblCntFinal].width =
                           capsAuxCam->video_sizes_tbl[j].width;
                        capsConsolidated.video_sizes_tbl[videoSizesTblCntFinal].height =
                           capsMainCam->video_sizes_tbl[j].height;
                    }
                    ++videoSizesTblCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.video_sizes_tbl_cnt = videoSizesTblCntFinal;

        // Consolidate livesnapshot sizes
        uint32_t livesnapshotSizesTblCntMain  = capsMainCam->livesnapshot_sizes_tbl_cnt;
        uint32_t livesnapshotSizesTblCntAux   = capsAuxCam->livesnapshot_sizes_tbl_cnt;
        uint32_t livesnapshotSizesTblCntFinal = 0;

        for (uint32_t i = 0; i < livesnapshotSizesTblCntMain; ++i) {
            for (uint32_t j = 0; j < livesnapshotSizesTblCntAux; ++j) {
                if ((capsMainCam->livesnapshot_sizes_tbl[i].width ==
                     capsAuxCam->livesnapshot_sizes_tbl[j].width) &&
                    (capsMainCam->livesnapshot_sizes_tbl[i].height ==
                     capsAuxCam->livesnapshot_sizes_tbl[j].height)) {
                    if (livesnapshotSizesTblCntFinal != i) {
                       capsConsolidated.livesnapshot_sizes_tbl[livesnapshotSizesTblCntFinal].width=
                          capsAuxCam->livesnapshot_sizes_tbl[j].width;
                       capsConsolidated.livesnapshot_sizes_tbl[livesnapshotSizesTblCntFinal].height=
                          capsMainCam->livesnapshot_sizes_tbl[j].height;
                    }
                    ++livesnapshotSizesTblCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.livesnapshot_sizes_tbl_cnt = livesnapshotSizesTblCntFinal;

        // Consolidate picture size
        // Find max picture dimension for main camera
        cam_dimension_t maxPicDimMain;
        maxPicDimMain.width  = 0;
        maxPicDimMain.height = 0;

        for(uint32_t i = 0; i < (capsMainCam->picture_sizes_tbl_cnt - 1); ++i) {
            if ((maxPicDimMain.width * maxPicDimMain.height) <
                    (capsMainCam->picture_sizes_tbl[i].width *
                            capsMainCam->picture_sizes_tbl[i].height)) {
                maxPicDimMain.width  = capsMainCam->picture_sizes_tbl[i].width;
                maxPicDimMain.height = capsMainCam->picture_sizes_tbl[i].height;
            }
        }

        // Find max picture dimension for aux camera
        cam_dimension_t maxPicDimAux;
        maxPicDimAux.width  = 0;
        maxPicDimAux.height = 0;

        for(uint32_t i = 0; i < (capsAuxCam->picture_sizes_tbl_cnt - 1); ++i) {
            if ((maxPicDimAux.width * maxPicDimAux.height) <
                    (capsAuxCam->picture_sizes_tbl[i].width *
                            capsAuxCam->picture_sizes_tbl[i].height)) {
                maxPicDimAux.width  = capsAuxCam->picture_sizes_tbl[i].width;
                maxPicDimAux.height = capsAuxCam->picture_sizes_tbl[i].height;
            }
        }

        // Choose the smaller of the two max picture dimensions
        if ((maxPicDimAux.width * maxPicDimAux.height) <
                (maxPicDimMain.width * maxPicDimMain.height)) {
            capsConsolidated.picture_sizes_tbl_cnt = capsAuxCam->picture_sizes_tbl_cnt;
            memcpy(capsConsolidated.picture_sizes_tbl, capsAuxCam->picture_sizes_tbl,
                    (capsAuxCam->picture_sizes_tbl_cnt * sizeof(cam_dimension_t)));
        }

        // Consolidate supported preview formats
        uint32_t supportedPreviewFmtCntMain  = capsMainCam->supported_preview_fmt_cnt;
        uint32_t supportedPreviewFmtCntAux   = capsAuxCam->supported_preview_fmt_cnt;
        uint32_t supportedPreviewFmtCntFinal = 0;
        for (uint32_t i = 0; i < supportedPreviewFmtCntMain; ++i) {
            for (uint32_t j = 0; j < supportedPreviewFmtCntAux; ++j) {
                if (capsMainCam->supported_preview_fmts[i] ==
                        capsAuxCam->supported_preview_fmts[j]) {
                    if (supportedPreviewFmtCntFinal != i) {
                        capsConsolidated.supported_preview_fmts[supportedPreviewFmtCntFinal] =
                            capsAuxCam->supported_preview_fmts[j];
                    }
                    ++supportedPreviewFmtCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.supported_preview_fmt_cnt = supportedPreviewFmtCntFinal;

        // Consolidate supported picture formats
        uint32_t supportedPictureFmtCntMain  = capsMainCam->supported_picture_fmt_cnt;
        uint32_t supportedPictureFmtCntAux   = capsAuxCam->supported_picture_fmt_cnt;
        uint32_t supportedPictureFmtCntFinal = 0;
        for (uint32_t i = 0; i < supportedPictureFmtCntMain; ++i) {
            for (uint32_t j = 0; j < supportedPictureFmtCntAux; ++j) {
                if (capsMainCam->supported_picture_fmts[i] ==
                        capsAuxCam->supported_picture_fmts[j]) {
                    if (supportedPictureFmtCntFinal != i) {
                        capsConsolidated.supported_picture_fmts[supportedPictureFmtCntFinal] =
                            capsAuxCam->supported_picture_fmts[j];
                    }
                    ++supportedPictureFmtCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.supported_picture_fmt_cnt = supportedPictureFmtCntFinal;
    }
    return capsConsolidated;
}


/*===========================================================================
 * FUNCTION   : translateInputParams
 *
 * DESCRIPTION: Translate a subset of input parameters from main camera. As main
 *              and aux cameras have different properties/params, this translation
 *              is needed before the input parameters are sent to the aux camera.
 *
 * PARAMETERS :
 * @paramsMainCam : Input parameters for main camera
 * @paramsAuxCam  : Input parameters for aux camera
 *
 * RETURN :
 * NO_ERROR           : Success
 * INVALID_OPERATION  : Failure
 *
 *==========================================================================*/
int32_t QCameraFOVControl::translateInputParams(parm_buffer_t* paramsMainCam,
                                                parm_buffer_t *paramsAuxCam)
{
    int32_t rc = INVALID_OPERATION;
    if (paramsMainCam && paramsAuxCam) {
        // First copy all the parameters from main to aux and then translate the subset
        memcpy(paramsAuxCam, paramsMainCam, sizeof(parm_buffer_t));

        // Translate zoom
        if (paramsMainCam->is_valid[CAM_INTF_PARM_ZOOM]) {
            uint32_t userZoom = 0;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_ZOOM, userZoom);
            convertUserZoomToMainAndAux(userZoom);
            ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_PARM_ZOOM,
                                            mFovControlData.zoomAux);
        }

        if (paramsMainCam->is_valid[CAM_INTF_PARM_AF_ROI] ||
            paramsMainCam->is_valid[CAM_INTF_PARM_AEC_ROI]) {
            convertDisparityForInputParams();
        }

        // Translate focus areas
        if (paramsMainCam->is_valid[CAM_INTF_PARM_AF_ROI]) {
            cam_roi_info_t roiAfMain;
            cam_roi_info_t roiAfAux;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_AF_ROI, roiAfMain);
            if (roiAfMain.num_roi > 0) {
                roiAfAux = translateFocusAreas(roiAfMain);
                ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_PARM_AF_ROI, roiAfAux);
            }
        }

        // Translate metering areas
        if (paramsMainCam->is_valid[CAM_INTF_PARM_AEC_ROI]) {
            cam_set_aec_roi_t roiAecMain;
            cam_set_aec_roi_t roiAecAux;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_AEC_ROI, roiAecMain);
            if (roiAecMain.aec_roi_enable == CAM_AEC_ROI_ON) {
                roiAecAux = translateMeteringAreas(roiAecMain);
                ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_PARM_AEC_ROI, roiAecAux);
            }
        }

        // Book-keep the preview dimension
        if (paramsMainCam->is_valid[CAM_INTF_META_STREAM_INFO]) {
            cam_stream_size_info_t camStreamInfo;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_META_STREAM_INFO, camStreamInfo);
            for (int i = 0; i < MAX_NUM_STREAMS; ++i) {
                if (camStreamInfo.type[i] == CAM_STREAM_TYPE_PREVIEW) {
                    mFovControlData.previewSize = camStreamInfo.stream_sizes[i];
                    break;
                }
            }
        }
        rc = NO_ERROR;
    }
    return rc;
}


/*===========================================================================
 * FUNCTION   : processResultMetadata
 *
 * DESCRIPTION: Process the metadata from main and aux cameras to generate the
 *              result metadata. The result metadata should be the metadata
 *              coming from the master camera. If aux camera is master, the
 *              subset of the metadata needs to be translated to main as that's
 *              the only camera seen by the application.
 *
 * PARAMETERS :
 * @metaMain  : metadata for main camera
 * @metaAux   : metadata for aux camera
 *
 * RETURN :
 * Result metadata for the logical camera. After successfully processing main
 * and aux metadata, the result metadata points to either main or aux metadata
 * based on which one was the master. In case of failure, it returns NULL.
 *==========================================================================*/
metadata_buffer_t* QCameraFOVControl::processResultMetadata(
                                                    metadata_buffer_t*  metaMain,
                                                    metadata_buffer_t*  metaAux)
{
    metadata_buffer_t* metaResult = NULL;

    if (metaMain || metaAux) {
        // Temporary code to determine the master camera.
        // This code will be replaced once we have the logic
        // to determine master based on the frame number in HAL.
        cam_type master = (cam_type)mFovControlResult.camMasterPreview;

        if ((master == CAM_TYPE_AUX) && metaAux) {
            // Translate face detection ROI
            IF_META_AVAILABLE(cam_face_detection_data_t, metaFD,
                                    CAM_INTF_META_FACE_DETECTION, metaAux) {
                cam_face_detection_data_t metaFDTranslated;
                metaFDTranslated = translateRoiFD(*metaFD);
                ADD_SET_PARAM_ENTRY_TO_BATCH(metaAux, CAM_INTF_META_FACE_DETECTION,
                                                metaFDTranslated);
            }
            metaResult = metaAux;
        } else if (metaMain) {
            metaResult = metaMain;
        } else {
            // Metadata for the master camera was dropped
            metaResult = NULL;
            return metaResult;
        }

        // Book-keep the needed metadata from main camera and aux camera
        mMutex.lock();
        if (metaMain) {
            IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, metaMain) {
                if (((*afState) == CAM_AF_STATE_FOCUSED_LOCKED)     ||
                    ((*afState) == CAM_AF_STATE_NOT_FOCUSED_LOCKED) ||
                    ((*afState) == CAM_AF_STATE_PASSIVE_FOCUSED)    ||
                    ((*afState) == CAM_AF_STATE_PASSIVE_UNFOCUSED)) {
                    mFovControlData.status3A.camMain.statusAF = AF_VALID;
                } else {
                    mFovControlData.status3A.camMain.statusAF = AF_INVALID;
                }
            }
            // TODO : Copy the spatial alignment metadata to mFovControlData.spatialAlign
            // Code for spatial alignment lib / metadata is not added yet
        }
        if (metaAux) {
            IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, metaAux) {
                if (((*afState) == CAM_AF_STATE_FOCUSED_LOCKED)     ||
                    ((*afState) == CAM_AF_STATE_NOT_FOCUSED_LOCKED) ||
                    ((*afState) == CAM_AF_STATE_PASSIVE_FOCUSED)    ||
                    ((*afState) == CAM_AF_STATE_PASSIVE_UNFOCUSED)) {
                    mFovControlData.status3A.camAux.statusAF = AF_VALID;
                } else {
                    mFovControlData.status3A.camAux.statusAF = AF_INVALID;
                }
            }
            // TODO : Copy the spatial alignment metadata to mFovControlData.spatialAlign
            // Code for spatial alignment lib / metadata is not added yet
        }
        mMutex.unlock();
    }
    return metaResult;
}


/*===========================================================================
 * FUNCTION   : getFovControlResult
 *
 * DESCRIPTION: Return the result from FOV control
 *
 * PARAMETERS : None
 *
 * RETURN     : FOV-control result
 *
 *==========================================================================*/
fov_control_result_t QCameraFOVControl::getFovControlResult()
{
    cam_sync_type_t camWide = mFovControlData.camWide;
    cam_sync_type_t camTele = mFovControlData.camTele;

    float zoom = findZoomRatio(mFovControlData.zoomMain) / 4096.0f;

    // Read AF status with mutex lock
    mMutex.lock();
    af_status afStatusMain = mFovControlData.status3A.camMain.statusAF;
    af_status afStatusAux  = mFovControlData.status3A.camAux.statusAF;
    mMutex.unlock();

    // Update the dual camera state based on the current zoom
    switch (mFovControlData.camState) {
        case STATE_WIDE:
            if (zoom >= mFovControlData.transitionParams.transitionLow) {
                mFovControlData.camState = STATE_TRANSITION_WIDE_TO_TELE;
            }
            break;
        case STATE_TRANSITION_WIDE_TO_TELE:
            if ((zoom < mFovControlData.transitionParams.transitionLow) &&
                (mFovControlResult.camMasterPreview == camWide) &&
                (mFovControlResult.camMaster3A      == camWide)) {
                mFovControlData.camState = STATE_WIDE;
            } else if ((zoom >= mFovControlData.transitionParams.transitionHigh) &&
                (mFovControlResult.camMasterPreview == camTele) &&
                (mFovControlResult.camMaster3A      == camTele) &&
                (afStatusAux == AF_VALID)) {
                mFovControlData.camState = STATE_TELE;
            }
            break;
        case STATE_TELE:
            if (zoom < mFovControlData.transitionParams.transitionHigh) {
                mFovControlData.camState = STATE_TRANSITION_TELE_TO_WIDE;
            }
            break;
        case STATE_TRANSITION_TELE_TO_WIDE:
            if ((zoom >= mFovControlData.transitionParams.transitionHigh) &&
                (mFovControlResult.camMasterPreview == camTele) &&
                (mFovControlResult.camMaster3A      == camTele)) {
                mFovControlData.camState = STATE_TELE;
            } else if ((zoom < mFovControlData.transitionParams.transitionLow) &&
                (mFovControlResult.camMasterPreview == camWide) &&
                (mFovControlResult.camMaster3A      == camWide) &&
                (afStatusMain == AF_VALID)) {
                mFovControlData.camState = STATE_WIDE;
            }
            break;
    }

    // Generate the result using updated dual camera state
    switch (mFovControlData.camState) {
        case STATE_WIDE:
            mFovControlResult.camState         = camWide;
            mFovControlResult.camMaster3A      = camWide;
            mFovControlResult.camMasterPreview = camWide;
            mFovControlResult.snapshotFusion   = false;
            break;
        case STATE_TRANSITION_WIDE_TO_TELE:
            if (zoom > mFovControlData.transitionParams.cutOverMainToAux) {
                mFovControlResult.camMasterPreview = camTele;
                mFovControlResult.camMaster3A      = camTele;
            } else {
                mFovControlResult.camMasterPreview = camWide;
                mFovControlResult.camMaster3A      = camWide;
            }
            mFovControlResult.camState       = (camWide | camTele);
            mFovControlResult.snapshotFusion = false;
            break;
        case STATE_TRANSITION_TELE_TO_WIDE:
            if (zoom < mFovControlData.transitionParams.cutOverAuxToMain) {
                mFovControlResult.camMasterPreview = camWide;
                mFovControlResult.camMaster3A      = camWide;
            } else {
                mFovControlResult.camMasterPreview = camTele;
                mFovControlResult.camMaster3A      = camTele;
            }
            mFovControlResult.camState       = (camWide | camTele);
            mFovControlResult.snapshotFusion = false;
            break;
        case STATE_TELE:
            mFovControlResult.camMaster3A      = camTele;
            mFovControlResult.camMasterPreview = camTele;
            mFovControlResult.camState         = camTele;
            mFovControlResult.snapshotFusion   = false;
            break;
    }

    // Debug print for the FOV-control result
    LOGD("Effective zoom: %f", zoom);
    LOGD("ZoomMain: %d, ZoomAux: %d", mFovControlData.zoomMain, mFovControlData.zoomAux);
    LOGD("Master camera for preview: %s",
        (mFovControlResult.camMasterPreview == CAM_TYPE_MAIN ) ? "Main" : "Aux");
    LOGD("Master camera for 3A     : %s",
        (mFovControlResult.camMaster3A == CAM_TYPE_MAIN ) ? "Main" : "Aux");
    LOGD("Main camera status: %s",
        (mFovControlResult.camState & CAM_TYPE_MAIN) ? "Active" : "Standby");
    LOGD("Aux camera status : %s",
        (mFovControlResult.camState & CAM_TYPE_AUX) ? "Active" : "Standby");
    LOGD("snapshot fusion   : %d", mFovControlResult.snapshotFusion);

    return mFovControlResult;
}


/*===========================================================================
 * FUNCTION   : calculateBasicFovRatio
 *
 * DESCRIPTION: Calculate the FOV ratio between main and aux cameras
 *
 * PARAMETERS : None
 *
 * RETURN     : Calculated FOV ratio
 *
 *==========================================================================*/
float QCameraFOVControl::calculateBasicFovRatio()
{
    float fovMain;
    float fovAux;
    float ret = 0.0f;

    if ((mDualCamParams.paramsMain.focalLengthMm > 0.0f) &&
         (mDualCamParams.paramsAux.focalLengthMm > 0.0f)) {
        fovMain = (mDualCamParams.paramsMain.sensorStreamWidth *
                    mDualCamParams.paramsMain.pixelPitchUm) /
                    mDualCamParams.paramsMain.focalLengthMm;

        fovAux  = (mDualCamParams.paramsAux.sensorStreamWidth *
                    mDualCamParams.paramsAux.pixelPitchUm) /
                    mDualCamParams.paramsAux.focalLengthMm;
        if (fovAux > 0.0f) {
            ret = (fovMain / fovAux);
        }
    }
    return ret;
}


/*===========================================================================
 * FUNCTION   : calculateFovAdjustmentWorstCaseDisparity
 *
 * DESCRIPTION: Calculate the FOV adjustment with worst case disparity
 *
 * PARAMETERS : None
 *
 * RETURN     : FOV adjustment
 *
 *==========================================================================*/
float QCameraFOVControl::calculateFovAdjustmentWorstCaseDisparity()
{
    float focalLengthAuxPixels;
    float focalLengthMainPixels;
    float focalLengthNormalized;
    float disparityBasic;
    float disparityFinal;
    float fovAdjustFromDisparity = 0.0f;

    if ((mDualCamParams.paramsMain.pixelPitchUm > 0.0f) &&
         (mDualCamParams.paramsAux.pixelPitchUm > 0.0f) &&
         (mDualCamParams.minFocusDistanceCm > 0.0f)) {
        focalLengthMainPixels = (mDualCamParams.paramsMain.focalLengthMm * 1000) /
                                mDualCamParams.paramsMain.pixelPitchUm;
        focalLengthAuxPixels  = (mDualCamParams.paramsAux.focalLengthMm * 1000) /
                                (mDualCamParams.paramsAux.pixelPitchUm);
        if ((focalLengthMainPixels > 0.0f) &&
            (focalLengthAuxPixels  > 0.0f)) {
            focalLengthNormalized  = focalLengthAuxPixels / focalLengthMainPixels;
            disparityBasic         = focalLengthMainPixels * mDualCamParams.baselineMm /
                                    (mDualCamParams.minFocusDistanceCm * 10);
            disparityFinal         = focalLengthNormalized * disparityBasic;

            if (mDualCamParams.paramsAux.sensorStreamWidth) {
                fovAdjustFromDisparity = 1 +
                    (disparityFinal / mDualCamParams.paramsAux.sensorStreamWidth);
            }
        }
    }
    return fovAdjustFromDisparity;
}


/*===========================================================================
 * FUNCTION   : calculateFovAdjustmentRollPitchYaw
 *
 * DESCRIPTION: Calculate FOV adjustment factor considering roll, pitch, yaw.
 *
 * PARAMETERS : none
 *
 * RETURN     : FOV adjustment factor
 *
 *==========================================================================*/
float QCameraFOVControl::calculateFovAdjustmentRollPitchYaw()
{
    // TODO :Change the logic here when needed. 7% is fine based on
    // the different configurations observed
    return (1.07);
}


/*===========================================================================
 * FUNCTION   : combineFovAdjustment
 *
 * DESCRIPTION: Calculate the final FOV adjustment by combining three FOV
 *              adjustments calculated earlier
 *
 * PARAMETERS :
 *  @fovAdjustBasic            : Basic FOV ratio
 *  @fovAdjustFromDisparity    : FOV adjustment due to disparity
 *  @fovAdjustFromRollPitchYaw : FOV adjustment with roll, pitch, yaw
 *
 * RETURN     : Final FOV adjustment value
 *
 *==========================================================================*/
float QCameraFOVControl::combineFovAdjustment(float fovAdjustBasic,
                                              float fovAdjustFromDisparity,
                                              float fovAdjustFromRollPitchYaw)
{
    return (fovAdjustBasic * fovAdjustFromDisparity * fovAdjustFromRollPitchYaw);
}


/*===========================================================================
 * FUNCTION   : calculateDualCamTransitionParams
 *
 * DESCRIPTION: Calculate the transition parameters needed to switch the camera
 *              between main and aux
 *
 * PARAMETERS :
 * @fovAdjustBasic       : basic FOV ratio
 * @zoomTranslationFactor: translation factor for main, aux zoom
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCameraFOVControl::calculateDualCamTransitionParams(
        float fovAdjustBasic,
        float zoomTranslationFactor)
{
    mFovControlData.transitionParams.cropRatio      = fovAdjustBasic;
    mFovControlData.transitionParams.cutOverFactor  = zoomTranslationFactor;

    mFovControlData.transitionParams.cutOverMainToAux = zoomTranslationFactor +
                        (mFovControlConfig.percentMarginHysterisis / 100.0) * fovAdjustBasic;
    mFovControlData.transitionParams.transitionHigh =
            mFovControlData.transitionParams.cutOverMainToAux +
            (mFovControlConfig.percentMarginMain / 100.0) * fovAdjustBasic;

    mFovControlData.transitionParams.cutOverAuxToMain = zoomTranslationFactor;
    mFovControlData.transitionParams.transitionLow =
            mFovControlData.transitionParams.cutOverAuxToMain -
            (mFovControlConfig.percentMarginAux / 100.0) * fovAdjustBasic;
}


/*===========================================================================
 * FUNCTION   : findZoomValue
 *
 * DESCRIPTION: For the input zoom ratio, find the zoom value.
 *              Zoom table contains zoom ratios where the indices
 *              in the zoom table indicate the corresponding zoom values.
 * PARAMETERS :
 * @zoomRatio : Zoom ratio
 *
 * RETURN     : Zoom value
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::findZoomValue(uint32_t zoomRatio)
{
    uint32_t zoom = 0;
    for (int i = 0; i < ZOOM_TABLE_SIZE; ++i) {
        if (zoomRatio <= zoomTableDualCam[i]) {
            zoom = i;
            break;
        }
    }
    return zoom;
}


/*===========================================================================
 * FUNCTION   : findZoomRatio
 *
 * DESCRIPTION: For the input zoom value, find the zoom ratio.
 *              Zoom table contains zoom ratios where the indices
 *              in the zoom table indicate the corresponding zoom values.
 * PARAMETERS :
 * @zoom      : zoom value
 *
 * RETURN     : zoom ratio
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::findZoomRatio(uint32_t zoom)
{
    return zoomTableDualCam[zoom];
}


/*===========================================================================
 * FUNCTION   : readjustZoomForAux
 *
 * DESCRIPTION: Calculate the zoom value for the aux camera based on the input
 *              zoom value for the main camera
 *
 * PARAMETERS :
 * @zoom      : Zoom value for main camera
 *
 * RETURN     : Zoom value for aux camera
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::readjustZoomForAux(uint32_t zoomMain)
{
    uint32_t zoomRatioMain;
    uint32_t zoomRatioAux;

    zoomRatioMain = findZoomRatio(zoomMain);
    zoomRatioAux  = zoomRatioMain / mFovControlData.transitionParams.cutOverFactor;

    return(findZoomValue(zoomRatioAux));
}


/*===========================================================================
 * FUNCTION   : readjustZoomForMain
 *
 * DESCRIPTION: Calculate the zoom value for the main camera based on the input
 *              zoom value for the aux camera
 *
 * PARAMETERS :
 * @zoom      : Zoom value for aux camera
 *
 * RETURN     : Zoom value for main camera
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::readjustZoomForMain(uint32_t zoomAux)
{
    uint32_t zoomRatioMain;
    uint32_t zoomRatioAux;

    zoomRatioAux  = findZoomRatio(zoomAux);
    zoomRatioMain = zoomRatioAux * mFovControlData.transitionParams.cutOverFactor;

    return(findZoomValue(zoomRatioMain));
}


/*===========================================================================
 * FUNCTION   : convertUserZoomToMainAndAux
 *
 * DESCRIPTION: Calculate the zoom value for the main and aux cameras based on
 *              the input user zoom value
 *
 * PARAMETERS :
 * @zoom      : User zoom value
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCameraFOVControl::convertUserZoomToMainAndAux(uint32_t zoom)
{
    mFovControlData.zoomMain = zoom;
    mFovControlData.zoomAux  = readjustZoomForAux(mFovControlData.zoomMain);
}


/*===========================================================================
 * FUNCTION   : convertDisparityForInputParams
 *
 * DESCRIPTION: Convert the disparity for translation of input parameters
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCameraFOVControl::convertDisparityForInputParams()
{
    Mutex::Autolock lock(mMutex);

    mFovControlData.shiftHorzAdjMain = mFovControlData.transitionParams.cropRatio /
            (mFovControlData.zoomMain / 4096.0f) * mFovControlData.spatialAlign.shiftHorz;
}


/*===========================================================================
 * FUNCTION   : translateFocusAreas
 *
 * DESCRIPTION: Translate the focus areas from main to aux camera.
 *
 * PARAMETERS :
 * @roiAfMain : Focus area ROI for main camera
 *
 * RETURN     : Translated focus area ROI for aux camera
 *
 *==========================================================================*/
cam_roi_info_t QCameraFOVControl::translateFocusAreas(cam_roi_info_t roiAfMain)
{
    float fovRatio;
    float AuxDiffRoiLeft;
    float AuxDiffRoiTop;
    float AuxRoiLeft;
    float AuxRoiTop;

    cam_roi_info_t roiAfAux;

    fovRatio = mFovControlData.transitionParams.cropRatio;

    for (int i = 0; i < roiAfMain.num_roi; ++i) {
        AuxDiffRoiLeft = fovRatio*(roiAfMain.roi[i].left -
                                (mFovControlData.previewSize.width / 2));
        AuxRoiLeft = (mFovControlData.previewSize.width / 2) + AuxDiffRoiLeft;

        AuxDiffRoiTop = fovRatio*(roiAfMain.roi[i].top -
                                        (mFovControlData.previewSize.height/ 2));
        AuxRoiTop = (mFovControlData.previewSize.height / 2) + AuxDiffRoiTop;

        roiAfAux.roi[i].width  *= fovRatio;
        roiAfAux.roi[i].height *= fovRatio;
        roiAfAux.roi[i].top     = AuxRoiTop;

        if (mDualCamParams.positionAux == CAM_POSITION_LEFT) {
            roiAfAux.roi[i].left = AuxRoiLeft + mFovControlData.shiftHorzAdjMain;
        } else {
            roiAfAux.roi[i].left = AuxRoiLeft - mFovControlData.shiftHorzAdjMain;
        }
    }
    return roiAfAux;
}


/*===========================================================================
 * FUNCTION   : translateMeteringAreas
 *
 * DESCRIPTION: Translate the AEC metering areas from main to aux camera.
 *
 * PARAMETERS :
 * @roiAfMain : AEC ROI for main camera
 *
 * RETURN     : Translated AEC ROI for aux camera
 *
 *==========================================================================*/
cam_set_aec_roi_t QCameraFOVControl::translateMeteringAreas(cam_set_aec_roi_t roiAecMain)
{
    float fovRatio;
    float AuxDiffRoiX;
    float AuxDiffRoiY;
    float AuxRoiX;
    float AuxRoiY;

    cam_set_aec_roi_t roiAecAux;

    fovRatio = mFovControlData.transitionParams.cropRatio;

    for (int i = 0; i < roiAecMain.num_roi; ++i) {
        AuxDiffRoiX = fovRatio*(roiAecMain.cam_aec_roi_position.coordinate[i].x -
                                        (mFovControlData.previewSize.width / 2));
        AuxRoiX = (mFovControlData.previewSize.width / 2) + AuxDiffRoiX;

        AuxDiffRoiY = fovRatio*(roiAecMain.cam_aec_roi_position.coordinate[i].y -
                                        (mFovControlData.previewSize.height / 2));
        AuxRoiY = (mFovControlData.previewSize.height / 2) + AuxDiffRoiY;

        roiAecAux.cam_aec_roi_position.coordinate[i].y = AuxRoiY;

        if (mDualCamParams.positionAux == CAM_POSITION_LEFT) {
            roiAecAux.cam_aec_roi_position.coordinate[i].x = AuxRoiX +
                                mFovControlData.shiftHorzAdjMain;
        } else {
            roiAecAux.cam_aec_roi_position.coordinate[i].x = AuxRoiX -
                                mFovControlData.shiftHorzAdjMain;
        }
    }
    return roiAecAux;
}


/*===========================================================================
 * FUNCTION   : translateRoiFD
 *
 * DESCRIPTION: Translate face detection ROI from aux metadata to main
 *
 * PARAMETERS :
 * @faceDetectionInfo  : face detection data from aux metadata. This face
 *                       detection data is overwritten with the translated
 *                       FD ROI.
 *
 * RETURN     : none
 *
 *==========================================================================*/
cam_face_detection_data_t QCameraFOVControl::translateRoiFD(cam_face_detection_data_t metaFD)
{
    cam_face_detection_data_t metaFDTranslated = metaFD;

    for (int i = 0; i < metaFDTranslated.num_faces_detected; ++i) {
        if (mDualCamParams.positionAux == CAM_POSITION_LEFT) {
            metaFDTranslated.faces[i].face_boundary.left -=
                mFovControlData.spatialAlign.shiftHorz;
        } else {
            metaFDTranslated.faces[i].face_boundary.left +=
                mFovControlData.spatialAlign.shiftHorz;
        }
    }
    return metaFDTranslated;
}
}; // namespace qcamera
