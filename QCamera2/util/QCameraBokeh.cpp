/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "QCameraBokeh"
// System dependencies
#include <dlfcn.h>
#include <utils/Errors.h>
#include <stdio.h>
#include <stdlib.h>
// Camera dependencies
#include "QCameraBokeh.h"
#include "QCameraTrace.h"
extern "C" {
#include "mm_camera_dbg.h"
}
//#include "dualcameraddm_appapi.h"
#include "dualcameraddm_wrapper.h"

const char* SCALE_CROP_ROTATION_FORMAT_STRING[] = {
        "Sensor Crop left = %d\n",
        "Sensor Crop top = %d\n",
        "Sensor Crop width = %d\n",
        "Sensor Crop height = %d\n",
        "Sensor ROI Map left = %d\n",
        "Sensor ROI Map top = %d\n",
        "Sensor ROI Map width = %d\n",
        "Sensor ROI Map height = %d\n",
        "CAMIF Crop left = %d\n",
        "CAMIF Crop top = %d\n",
        "CAMIF Crop width = %d\n",
        "CAMIF Crop height = %d\n",
        "CAMIF ROI Map left = %d\n",
        "CAMIF ROI Map top = %d\n",
        "CAMIF ROI Map width = %d\n",
        "CAMIF ROI Map height = %d\n",
        "ISP Crop left = %d\n",
        "ISP Crop top = %d\n",
        "ISP Crop width = %d\n",
        "ISP Crop height = %d\n",
        "ISP ROI Map left = %d\n",
        "ISP ROI Map top = %d\n",
        "ISP ROI Map width = %d\n",
        "ISP ROI Map height = %d\n",
        "CPP Crop left = %d\n",
        "CPP Crop top = %d\n",
        "CPP Crop width = %d\n",
        "CPP Crop height = %d\n",
        "CPP ROI Map left = %d\n",
        "CPP ROI Map top = %d\n",
        "CPP ROI Map width = %d\n",
        "CPP ROI Map height = %d\n",
        "Focal length Ratio = %f\n",
        "Current pipeline mirror flip setting = %d\n",
        "Current pipeline rotation setting = %d\n"
};

const char* CALIB_FMT_STRINGS[] = {
    "Calibration OTP format version = %d\n",
    "Main Native Sensor Resolution width = %dpx\n",
    "Main Native Sensor Resolution height = %dpx\n",
    "Main Calibration Resolution width = %dpx\n",
    "Main Calibration Resolution height = %dpx\n",
    "Main Focal length ratio = %f\n",
    "Aux Native Sensor Resolution width = %dpx\n",
    "Aux Native Sensor Resolution height = %dpx\n",
    "Aux Calibration Resolution width = %dpx\n",
    "Aux Calibration Resolution height = %dpx\n",
    "Aux Focal length ratio = %f\n",
    "Relative Rotation matrix [0] through [8] = %s\n",
    "Relative Geometric surface parameters [0] through [31] = %s\n",
    "Relative Principal point X axis offset (ox) = %fpx\n",
    "Relative Principal point Y axis offset (oy) = %fpx\n",
    "Relative position flag = %d\n",
    "Baseline distance = %fmm\n",
    "Main sensor mirror and flip setting = %d\n",
    "Aux sensor mirror and flip setting = %d\n",
    "Module orientation during calibration = %d\n",
    "Rotation flag = %d\n",
    "Main Normalized Focal length = %fpx\n",
    "Aux Normalized Focal length = %fpx"
};

#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while (0)

#define DUMP(fmt, args...)                           \
{                                                    \
    if (m_bDebug) {                                  \
        mDebugData.appendFormat(fmt, ##args);        \
    }                                                \
}

#define FDUMP(file, string, idx)                     \
{                                                    \
    if (m_bDebug) {                                  \
        dumpInputParams(file, string, idx);          \
    }                                                \
}


namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraBokeh
 *
 * DESCRIPTION: constructor of QCameraBokeh.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBokeh::QCameraBokeh() : QCameraHALPP()
{
    m_dlHandle = NULL;
    m_pCaps = NULL;
    memset(&mBokehData, 0, sizeof(mBokehData));
    bNeedCamSwap = false;
    m_bDebug = false;
}

/*===========================================================================
 * FUNCTION   : ~QCameraBokeh
 *
 * DESCRIPTION: destructor of QCameraBokeh.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraBokeh::~QCameraBokeh()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of QCameraBokeh
 *
 * PARAMETERS :
 *   @bufNotifyCb    : call back function after HALPP process
 *   @getOutputCb   : call back function to request output buffer
 *   @pUserData      : Parent of HALPP, i.e. QCameraPostProc
 *   @pStaticParam  : holds dual camera calibration data in an array and its size
 *                       (expected size is 264 bytes)
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::init(
        halPPBufNotify bufNotifyCb,
        halPPGetOutput getOutputCb,
        void *pUserData,
        void *pStaticParam)
{
    LOGH("E");
    int32_t rc = NO_ERROR;

    QCameraHALPP::init(bufNotifyCb, getOutputCb, pUserData);

    m_pCaps = (cam_capability_t *)pStaticParam;

    /* we should load 3rd libs here, with dlopen/dlsym */
    doBokehInit();

    /* To dump debug data */
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.bokeh.debug", prop, "0");
    m_bDebug = atoi(prop);

    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de initialization of QCameraBokeh
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::deinit()
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    m_dlHandle = NULL;

    QCameraHALPP::deinit();
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: starting QCameraBokeh
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::start()
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    rc = QCameraHALPP::start();

    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop QCameraBokeh
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::stop()
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    rc = QCameraHALPP::stop();
    mDebugData.clear();

    LOGH("X");
    return rc;
}


/*===========================================================================
 * FUNCTION   : feedInput
 *
 * DESCRIPTION: function to feed input data.
 *              Enqueue the frame index to inputQ if it is new frame
 *              Also, add the input image data to frame hash map
 *
 * PARAMETERS :
 *   @pInputData    : ptr to input data
 *   @bFeedOutput  : true if ready for feeding output
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::feedInput(qcamera_hal_pp_data_t *pInputData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (NULL != pInputData) {
        QCameraStream* pSnapshotStream = NULL;
        mm_camera_buf_def_t *pInputSnapshotBuf = getSnapshotBuf(pInputData, pSnapshotStream);
        if (pInputSnapshotBuf != NULL) {
            // Check for main and aux handles
            uint32_t mainHandle = get_main_camera_handle(
                    pInputData->frame->camera_handle);
            uint32_t auxHandle = get_aux_camera_handle(
                    pInputData->frame->camera_handle);
            LOGH("mainHandle = 0x%x, auxHandle = 0x%x", mainHandle, auxHandle);

            if ((!mainHandle) && (!auxHandle)) {
                // Both main and aux handles are not available
                // Return from here
                return BAD_VALUE;
            }
            if (mainHandle && (mBokehData.main_input == NULL)) {
                mBokehData.main_input = pInputData;
                //Encode main image always
                mBokehData.main_input->needEncode = true;
                LOGH("Update main input");
            }
            else if (auxHandle && (mBokehData.aux_input == NULL)) {
                mBokehData.aux_input = pInputData;
                LOGH("Update aux input");
            }
            // request output buffer only if both main and aux input data are recieved
            if ((mBokehData.aux_input != NULL) && (mBokehData.main_input != NULL)) {
                if (bNeedCamSwap) {
                    SWAP(mBokehData.main_input, mBokehData.aux_input);
                }
                m_halPPGetOutputCB(pInputSnapshotBuf->frame_idx, m_pHalPPMgr);
            }
        }
    } else {
        LOGE("pInput is NULL");
        rc = UNEXPECTED_NULL;
    }
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : feedOutput
 *
 * DESCRIPTION: function to feed output buffer and metadata
 *
 * PARAMETERS :
 *   @pOutput     : ptr to output data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::feedOutput(qcamera_hal_pp_data_t *pOutputData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (NULL == pOutputData) {
        LOGE("Error! pOutput hal pp data is NULL");
        return BAD_VALUE;
    }
    if (mBokehData.bokeh_output != NULL) {
        releaseData(pOutputData);
        LOGE("Error!! Output data is not empty");
        return BAD_VALUE;
    }
    rc = getOutputBuffer(mBokehData.main_input, pOutputData);
    if (rc == NO_ERROR) {
        LOGH("filling bokeh output %d", rc);
        mBokehData.bokeh_output = pOutputData;
    }

    LOGH("X rc: %d", rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : process
 *
 * DESCRIPTION: Start Bokeh process
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraBokeh::process()
{
    int32_t rc = NO_ERROR;

    // Start the blending process when it is ready
    if (canProcess()) {
        LOGH("Start Bokeh processing");
        QCameraStream* pAuxStream = NULL;
        QCameraStream* pMainStream = NULL;

        mm_camera_buf_def_t *pAuxSnap = getSnapshotBuf(mBokehData.aux_input, pAuxStream);
        mm_camera_buf_def_t *pMainSnap = getSnapshotBuf(mBokehData.main_input, pMainStream);

        if (!pAuxStream || !pMainStream || !pAuxSnap || !pMainSnap ||
                (mBokehData.bokeh_output->misc_buf == NULL) ||
                (mBokehData.bokeh_output->misc_buf->buffer == NULL)) {
            releaseData(mBokehData.aux_input);
            releaseData(mBokehData.main_input);
            releaseData(mBokehData.bokeh_output);
            LOGE("Error!! Snapshot buffer/stream or depthmap not available");
            return BAD_VALUE;
        }
        mm_camera_super_buf_t *pOutputSuperBuf = mBokehData.bokeh_output->frame;
        mm_camera_buf_def_t *pOutputBuf = pOutputSuperBuf->bufs[0];

        // Get offset info from reproc stream
        cam_frame_len_offset_t offset;
        memset(&offset, 0, sizeof(offset));
        pMainStream->getFrameOffset(offset);

        //Get input and output parameter
        bokeh_input_params_t inParams;
        getInputParams(inParams);
        uint8_t * pDepthMap = (uint8_t *)mBokehData.bokeh_output->misc_buf->buffer;

        rc = doBokehProcess(
                (const uint8_t *)pMainSnap->buffer,
                (const uint8_t *)pAuxSnap->buffer,
                inParams, (uint8_t *)pOutputBuf->buffer, pDepthMap);

        if (rc != NO_ERROR) {
            LOGE("Error in bokeh processing. Fallback and copy input to output");
            memcpy(pOutputBuf->buffer, pMainSnap->buffer, offset.frame_len);
        }

        if (m_bDebug) {
            dumpYUVtoFile((uint8_t *)pAuxSnap->buffer, inParams.aux.offset,
                    pAuxSnap->frame_idx, "Aux");
            dumpYUVtoFile((uint8_t *)pMainSnap->buffer, inParams.main.offset,
                    pMainSnap->frame_idx,  "Main");
            dumpYUVtoFile((uint8_t *)pOutputBuf->buffer, inParams.bokehOut.offset,
                    pAuxSnap->frame_idx, "BokehOutput");
            dumpYUVtoFile(pDepthMap, inParams.depth.offset,
                    pAuxSnap->frame_idx, "DepthMap");
            dumpInputParams("input_params", mDebugData, pMainSnap->frame_idx);
        }

        // Invalidate input buffer
        QCameraMemory *pMem = NULL;
        pMem = (QCameraMemory *)pAuxSnap->mem_info;
        pMem->invalidateCache(pAuxSnap->buf_idx);
        pMem = (QCameraMemory *)pMainSnap->mem_info;
        pMem->invalidateCache(pMainSnap->buf_idx);
        // Clean and invalidate output buffer
        mBokehData.bokeh_output->snapshot_heap->cleanInvalidateCache(0);

        // Callback Manager to notify output buffer and return input buffers
        LOGH("notifying Bokeh output");
        m_halPPBufNotifyCB(mBokehData.bokeh_output, m_pHalPPMgr);
        LOGH("CB for main input");
        m_halPPBufNotifyCB(mBokehData.main_input, m_pHalPPMgr);
        LOGH("CB for aux input");
        m_halPPBufNotifyCB(mBokehData.aux_input, m_pHalPPMgr);

        // Once process is complete, reset context data
        // Post proc would take care of releasing the data
        memset(&mBokehData, 0, sizeof(mBokehData));
    }
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : canProcess
 *
 * DESCRIPTION: function to release internal resources
 * RETURN     : If Bokeh module can process
 *==========================================================================*/
bool QCameraBokeh::canProcess()
{
    LOGD("E");
    bool ready = false;
    //Check if we have all input and output buffers
    if (mBokehData.main_input && mBokehData.aux_input && mBokehData.bokeh_output) {
        ready = true;
        LOGH("ready: %d", ready);
    }
    LOGD("X");
    return ready;
}

/*===========================================================================
 * FUNCTION   : getInputParams
 *
 * DESCRIPTION: Helper function to get input params from input metadata
 *==========================================================================*/
void QCameraBokeh::getInputParams(bokeh_input_params_t& inParams)
{
    QCameraStream* pAuxStream = NULL;
    QCameraStream* pMainStream = NULL;
    QCameraStream* pAuxMetaStream  = NULL;
    QCameraStream* pMainMetaStream  = NULL;

    mm_camera_buf_def_t *pAuxSnap = getSnapshotBuf(mBokehData.aux_input, pAuxStream);
    mm_camera_buf_def_t *pMainSnap = getSnapshotBuf(mBokehData.main_input, pMainStream);
    mm_camera_buf_def_t *pAuxMeta = getMetadataBuf(mBokehData.aux_input, pAuxMetaStream);
    mm_camera_buf_def_t *pMainMeta = getMetadataBuf(mBokehData.main_input, pMainMetaStream);

    if (!pAuxSnap || !pMainSnap || !pAuxMeta || !pMainMeta) {
        LOGH("NULL pointer pAuxSnap: %p, pMainSnap: %p pAuxMeta: %p, pMainMeta: %p",
                pAuxSnap, pMainSnap, pAuxMeta, pMainMeta);
        return;
    }

    if (!pAuxStream || !pMainStream || !pAuxMetaStream || !pMainMetaStream) {
        LOGH("NULL pointer pAuxStream: %p, pMainStream: %p pAuxMetaStream: %p,"
                "pMainMetaStream: %p", pAuxStream, pMainStream, pAuxMetaStream, pMainMetaStream);
        return;
    }

    metadata_buffer_t *pAuxMetaBuf = (metadata_buffer_t *)pAuxMeta->buffer;
    metadata_buffer_t *pMainMetaBuf = (metadata_buffer_t *)pMainMeta->buffer;

    // main frame size
    cam_frame_len_offset_t offset;
    pMainStream->getFrameOffset(offset);
    inParams.main.width     = offset.mp[0].width;
    inParams.main.height    = offset.mp[0].height;
    inParams.main.stride    = offset.mp[0].stride;
    inParams.main.scanline  = offset.mp[0].scanline;
    inParams.main.frame_len = offset.frame_len;
    inParams.main.offset = offset;
    DUMP("Primary width: %d height: %d stride[0]:%d scanline[0]:%d "
            "stride[1]:%d, scanline[1]:%d frame_len: %d",
            inParams.main.width, inParams.main.height,
            offset.mp[0].stride, offset.mp[0].scanline,
            offset.mp[1].stride, offset.mp[1].scanline,
            inParams.main.frame_len);

    // aux frame size
    pAuxStream->getFrameOffset(offset);
    inParams.aux.width     = offset.mp[0].width;
    inParams.aux.height    = offset.mp[0].height;
    inParams.aux.stride    = offset.mp[0].stride;
    inParams.aux.scanline  = offset.mp[0].scanline;
    inParams.aux.frame_len = offset.frame_len;
    inParams.aux.offset = offset;

    DUMP("\nAuxiliary width: %d height: %d stride[0]:%d scanline[0]:%d "
            "stride[1]:%d, scanline[1]:%d frame_len: %d\n",
            inParams.aux.width, inParams.aux.height,
            offset.mp[0].stride, offset.mp[0].scanline,
            offset.mp[1].stride, offset.mp[1].scanline,
            inParams.aux.frame_len);

    inParams.bokehOut = inParams.main;
    //will be filled up in doBokehProcess
    memset(&inParams.depth, 0, sizeof(inParams.depth));

    inParams.sAuxReprocessInfo = extractReprocessInfo(pAuxMetaBuf);
    inParams.sMainReprocessInfo = extractReprocessInfo(pMainMetaBuf);

    FDUMP("primary", inParams.sMainReprocessInfo, pMainSnap->frame_idx);
    FDUMP("auxiliary", inParams.sAuxReprocessInfo, pMainSnap->frame_idx);

    inParams.sCalibData = extractCalibrationData();
    FDUMP("otp", inParams.sCalibData, pMainSnap->frame_idx);


    IF_META_AVAILABLE(cam_rtb_blur_info_t, blurInfo,
            CAM_INTF_PARAM_BOKEH_BLUR_LEVEL, pMainMetaBuf) {
        inParams.blurLevel = (float) blurInfo->blur_level / blurInfo->blur_max_value;
        DUMP("\nBlurLevel = %f", inParams.blurLevel);
    }
    IF_META_AVAILABLE(cam_rect_t, hAfRegions, CAM_INTF_META_AF_DEFAULT_ROI, pMainMetaBuf) {
        inParams.afROI = *hAfRegions;
        DUMP("\nAF ROI : (%d, %d, %d, %d)",
                inParams.afROI.left, inParams.afROI.top,
                inParams.afROI.width, inParams.afROI.height);
    }
    IF_META_AVAILABLE(cam_stream_crop_info_t, ispCropInfo,
            CAM_INTF_META_SNAP_CROP_INFO_ISP, pMainMetaBuf) {
        inParams.afROIMap = ispCropInfo->roi_map;
        DUMP("\nAF ROI map: (%d, %d, %d, %d)",
                inParams.afROIMap.left, inParams.afROIMap.top,
                inParams.afROIMap.width, inParams.afROIMap.height);
    }

    return;
}


int32_t QCameraBokeh::doBokehInit()
{
    LOGH("E");
    int rc = NO_ERROR;
    if (!m_pCaps || !m_pCaps->aux_cam_cap || !m_pCaps->main_cam_cap) {
        return BAD_VALUE;
    }
    //Main is wide and Aux is Tele. Set primary as Tele and auxiliary as Wide to library.
    if (m_pCaps->aux_cam_cap->focal_length > m_pCaps->main_cam_cap->focal_length) {
        bNeedCamSwap = true;
        LOGH("Need to swap main and aux cams");
    }
    LOGH("X");
    return rc;
}

int32_t QCameraBokeh::doBokehProcess(
        const uint8_t* pMain,
        const uint8_t* pAux,
        bokeh_input_params_t &inParams,
        uint8_t* pOut,
        uint8_t* pDepthMap)
{
    LOGD(":E");
    ATRACE_BEGIN("doBokehProcess");
    int32_t rc = NO_ERROR;
    QCameraStream* pStream = NULL;
    uint32_t focusX,focusY;
    qrcp::DualCameraDDMEffects *effectObj = NULL;
    qrcp::DualCameraDDMEffects::EffectType type = qrcp::DualCameraDDMEffects::REFOCUS_CIRCLE;

    //1. get depth map size from lib
    cam_dimension_t dmSize;
    qrcp::getDepthMapSize(inParams.main.width, inParams.main.height,
            dmSize.width, dmSize.height);
    uint32_t depthLen = dmSize.width * dmSize.height;
    unsigned int depthStride = dmSize.width;
    inParams.depth.offset.num_planes = 1;
    inParams.depth.offset.mp[0].offset = 0;
    inParams.depth.offset.mp[0].width = inParams.depth.offset.mp[0].stride =
            inParams.depth.width = inParams.depth.stride = dmSize.width;
    inParams.depth.offset.mp[0].height = inParams.depth.offset.mp[0].scanline =
            inParams.depth.height = inParams.depth.scanline = dmSize.height;
    inParams.depth.frame_len = inParams.depth.offset.frame_len =
            inParams.depth.offset.mp[0].len = depthLen;
    DUMP("\nDepth map W %d H %d ", dmSize.width, dmSize.height);

    //2. generate depth map
    const uint8_t* primaryY = pMain;
    uint32_t main_uv_offset = inParams.main.offset.mp[0].len;
    const uint8_t* primaryVU = pMain + main_uv_offset;
    unsigned int primaryWidth = inParams.main.width;
    unsigned int primaryHeight = inParams.main.height;
    unsigned int primaryStrideY = inParams.main.stride;
    unsigned int primaryStrideVU = inParams.main.offset.mp[1].stride;

    const uint8_t* auxiliaryY = pAux;
    uint32_t aux_uv_offset = inParams.aux.offset.mp[0].len;
    const uint8_t* auxiliaryVU = pAux + aux_uv_offset;
    unsigned int auxiliaryWidth = inParams.aux.width;
    unsigned int auxiliaryHeight = inParams.aux.height;
    unsigned int auxiliaryStrideY = inParams.aux.stride;
    unsigned int auxiliaryStrideVU = inParams.aux.offset.mp[1].stride;

    cam_rect_t goodRoi = {0,0,0,0};
    const float focalLengthPrimaryCamera =
        MAX(m_pCaps->main_cam_cap->focal_length, m_pCaps->aux_cam_cap->focal_length);
    bool isAuxMono = (m_pCaps->aux_cam_cap->color_arrangement == CAM_FILTER_ARRANGEMENT_Y);
    qrcp::SensorConfiguration config =
            isAuxMono ? qrcp::SYMMETRIC_BAYER_MONO : qrcp::STANDARD_WIDE_BAYER_BAYER;

    qrcp::DepthMapMode ddmMode = qrcp::DepthMapMode::ENHANCED_MODE;
    if (QCameraCommon::is_target_SDM630())
        ddmMode = qrcp::DepthMapMode::NORMAL_MODE;

    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.bokeh.ddmmode", prop, "");
    if (strlen(prop) > 0) {
        if (!strcmp(prop, "normal"))
            ddmMode = qrcp::DepthMapMode::NORMAL_MODE;
        else if (!strcmp(prop, "enhanced"))
            ddmMode = qrcp::DepthMapMode::ENHANCED_MODE;
    }

    DUMP("\nDepthStride = %d \nfocalLengthPrimaryCamera = %f \n"
            "SensorConfiguration = %d", depthStride, focalLengthPrimaryCamera, config);
    DUMP("\nDepthmap mode = %s ", (ddmMode == qrcp::DepthMapMode::NORMAL_MODE) ?
            "normal" : "enhanced");

    LOGI("[KPI Perf]: PROFILE_BOKEH_PROCESS : E");
    qrcp::DDMWrapperStatus status = qrcp::dualCameraGenerateDDM(
            primaryY, primaryVU, primaryWidth, primaryHeight,
            primaryStrideY, primaryStrideVU,
            auxiliaryY, auxiliaryVU, auxiliaryWidth, auxiliaryHeight,
            auxiliaryStrideY, auxiliaryStrideVU,
            pDepthMap, depthStride,
            goodRoi.left, goodRoi.top, goodRoi.width,goodRoi.height,
            inParams.sMainReprocessInfo.string(), inParams.sAuxReprocessInfo.string(),
            inParams.sCalibData.string(), focalLengthPrimaryCamera, config, ddmMode);

    if (!status.ok()) {
        LOGE("depth map generation failed: %s, errorcode %d",
                status.getErrorMessage().c_str(), status.getErrorCode());
        rc = BAD_VALUE;
        goto done;
    }

    LOGH("Depth map generated successfully");
    DUMP("\nGood ROI : (%d, %d, %d, %d)",
            goodRoi.left, goodRoi.top, goodRoi.width,goodRoi.height);

    //3. Bokeh processing using above depth map
    if ((inParams.afROIMap.width != 0) && (inParams.afROI.width != 0) &&
            (inParams.afROIMap.height != 0) && (inParams.afROI.height != 0)) {
        //get center of AF ROI and scale it from sensor coordinates (ROImap) to goodROI.
        focusX = inParams.afROI.left + inParams.afROI.width/2;
        focusX = focusX * primaryWidth / inParams.afROIMap.width;
        focusX = (focusX - goodRoi.left) * goodRoi.width / primaryWidth;
        focusY = inParams.afROI.top + inParams.afROI.height/2;
        focusY = focusY * primaryHeight / inParams.afROIMap.height;
        focusY = (focusY - goodRoi.top) * goodRoi.height / primaryHeight;
    } else {
        LOGE("Error in getting ROI information from meta, default to center ROI");
        focusX = goodRoi.width / 2;
        focusY = goodRoi.height / 2;
    }
    DUMP("\nFocusPoint = (%d, %d)", focusX,focusY);
    DUMP("\nDestination W %d H %d ", goodRoi.width,goodRoi.height);
    effectObj = new qrcp::DualCameraDDMEffects(
        primaryY, primaryVU, primaryWidth, primaryHeight, primaryStrideY, primaryStrideVU,
        pDepthMap, dmSize.width, dmSize.height, depthStride,
        goodRoi.left, goodRoi.top, goodRoi.width,goodRoi.height,
        goodRoi.width, goodRoi.height);
    if (effectObj) {
        qrcp::DDMWrapperStatus status = effectObj->renderEffect(
            type, focusX, focusY, inParams.blurLevel,
            pOut, pOut + main_uv_offset, primaryStrideY, primaryStrideVU);
        if (!status.ok()) {
            LOGE("render failed: %s", status.getErrorMessage().c_str());
            rc = BAD_VALUE;
            goto done;
        }
        LOGH("Blur rendering successful");
    }

    LOGI("[KPI Perf]: PROFILE_BOKEH_PROCESS : X");

    //Bokeh output size will be goodROI.width X goodROI.height.
    //set stream crop info so that jpeg will crop and upscale to original image size.
    getSnapshotBuf(mBokehData.bokeh_output, pStream);
    cam_rect_t bokeh_out_dim;
    bokeh_out_dim.top = 0;
    bokeh_out_dim.left = 0;
    bokeh_out_dim.width = goodRoi.width;
    bokeh_out_dim.height = goodRoi.height;
    if (pStream != NULL) {
        pStream->setCropInfo(bokeh_out_dim);
    }

    //modify bokeh offset
    inParams.bokehOut.width = inParams.bokehOut.offset.mp[0].width = goodRoi.width;
    inParams.bokehOut.height = inParams.bokehOut.offset.mp[0].height = goodRoi.height;
    inParams.bokehOut.offset.mp[1].width = goodRoi.width;
    inParams.bokehOut.offset.mp[1].height = goodRoi.height/2;

done:
    if (effectObj) {
        delete effectObj;
    }
    ATRACE_END();
    LOGD("X");
    return rc;
}

void QCameraBokeh::dumpYUVtoFile(
        const uint8_t* pBuf,
        cam_frame_len_offset_t offset,
        uint32_t idx,
        const char* name_prefix)
{
    char filename[256];
    snprintf(filename, sizeof(filename), QCAMERA_DUMP_FRM_LOCATION"%s_%dx%d_%d.yuv",
                name_prefix, offset.mp[0].width, offset.mp[0].height, idx);

    int file_fd = open(filename, O_RDWR | O_CREAT, 0777);
    ssize_t written_len = 0;
    if (file_fd >= 0) {
        void *data = NULL;

        fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        for (uint32_t i = 0; i < offset.num_planes; i++) {
            uint32_t index = offset.mp[i].offset;
            if (i > 0) {
                index += offset.mp[i-1].len;
            }
            for (int j = 0; j < offset.mp[i].height; j++) {
                data = (void *)(pBuf + index);
                written_len += write(file_fd, data,
                        (size_t)offset.mp[i].width);
                index += (uint32_t)offset.mp[i].stride;
            }
        }
        close(file_fd);
    }

}

const char* QCameraBokeh::buildCommaSeparatedString(float array[], size_t length) {
    String8 str;
    str.appendFormat("%.4f", array[0]);
    for(size_t i = 1; i < length; i++) {
        str.appendFormat(",%.4f", array[i]);
    }
    return str.string();
}

String8 QCameraBokeh::flattenCropInfo(cam_stream_crop_info_t* crop, uint8_t index)
{
    String8 cropInfo;
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 0], crop->crop.left);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 1], crop->crop.top);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 2], crop->crop.width);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 3], crop->crop.height);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 4], crop->roi_map.left);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 5], crop->roi_map.top);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 6], crop->roi_map.width);
    cropInfo.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index + 7], crop->roi_map.height);

    return cropInfo;
}

String8 QCameraBokeh::extractReprocessInfo(metadata_buffer_t *metadata)
{
    String8 reprocBlob;
    uint8_t index = 0;
    IF_META_AVAILABLE(cam_stream_crop_info_t, sensorCropInfo,
            CAM_INTF_META_SNAP_CROP_INFO_SENSOR, metadata) {
        reprocBlob.append(flattenCropInfo(sensorCropInfo, index));
    }
    index += 8;

    IF_META_AVAILABLE(cam_stream_crop_info_t, camifCropInfo,
            CAM_INTF_META_SNAP_CROP_INFO_CAMIF, metadata) {
        reprocBlob.append(flattenCropInfo(camifCropInfo, index));
    }
    index += 8;

    IF_META_AVAILABLE(cam_stream_crop_info_t, ispCropInfo,
            CAM_INTF_META_SNAP_CROP_INFO_ISP, metadata) {
        reprocBlob.append(flattenCropInfo(ispCropInfo, index));
    }
    index += 8;

    IF_META_AVAILABLE(cam_stream_crop_info_t, cppCropInfo,
            CAM_INTF_META_SNAP_CROP_INFO_CPP, metadata) {
        reprocBlob.append(flattenCropInfo(cppCropInfo, index));
    }
    index += 8;

    IF_META_AVAILABLE(cam_focal_length_ratio_t, ratio,
            CAM_INTF_META_AF_FOCAL_LENGTH_RATIO, metadata) {
        reprocBlob.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index], ratio->focalLengthRatio);
    }
    index++;

    int flipValue = 0;
    IF_META_AVAILABLE(int32_t, flip, CAM_INTF_PARM_FLIP, metadata) {
        flipValue = *flip;
    } else {
        flipValue = m_pCaps->sensor_rotation;
    }
    reprocBlob.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index], flipValue);
    index++;

    IF_META_AVAILABLE(cam_rotation_info_t, rotationInfo,
            CAM_INTF_PARM_ROTATION, metadata) {
        uint8_t orientation = rotationInfo->rotation;
        int rotation = 0;
        if (orientation == ROTATE_0) {
           rotation = 0;
        } else if (orientation == ROTATE_90) {
           rotation = 90;
        } else if (orientation == ROTATE_180) {
           rotation = 180;
        } else if (orientation == ROTATE_270) {
           rotation = 270;
        }
        reprocBlob.appendFormat(SCALE_CROP_ROTATION_FORMAT_STRING[index], rotation);
    }
    return reprocBlob;
}

String8 QCameraBokeh::extractCalibrationData()
{
    String8 calibData;
    cam_related_system_calibration_data_t calib_data = m_pCaps->related_cam_calibration;

    if (bNeedCamSwap) {
        SWAP(calib_data.main_cam_specific_calibration, calib_data.aux_cam_specific_calibration);
        SWAP(calib_data.main_sensor_mirror_flip_setting, calib_data.aux_sensor_mirror_flip_setting);
    }
    calibData.appendFormat(CALIB_FMT_STRINGS[0], calib_data.calibration_format_version);
    calibData.appendFormat(CALIB_FMT_STRINGS[1],
            calib_data.main_cam_specific_calibration.native_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[2],
            calib_data.main_cam_specific_calibration.native_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[3],
            calib_data.main_cam_specific_calibration.calibration_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[4],
            calib_data.main_cam_specific_calibration.calibration_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[5],
            calib_data.main_cam_specific_calibration.focal_length_ratio);

    calibData.appendFormat(CALIB_FMT_STRINGS[6],
            calib_data.aux_cam_specific_calibration.native_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[7],
            calib_data.aux_cam_specific_calibration.native_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[8],
            calib_data.aux_cam_specific_calibration.calibration_sensor_resolution_width);
    calibData.appendFormat(CALIB_FMT_STRINGS[9],
            calib_data.aux_cam_specific_calibration.calibration_sensor_resolution_height);
    calibData.appendFormat(CALIB_FMT_STRINGS[10],
            calib_data.aux_cam_specific_calibration.focal_length_ratio);

    calibData.appendFormat(CALIB_FMT_STRINGS[11],
            buildCommaSeparatedString(calib_data.relative_rotation_matrix,
            RELCAM_CALIB_ROT_MATRIX_MAX));
    calibData.appendFormat(CALIB_FMT_STRINGS[12],
            buildCommaSeparatedString(calib_data.relative_geometric_surface_parameters,
            RELCAM_CALIB_SURFACE_PARMS_MAX));

    calibData.appendFormat(CALIB_FMT_STRINGS[13], calib_data.relative_principle_point_x_offset);
    calibData.appendFormat(CALIB_FMT_STRINGS[14], calib_data.relative_principle_point_y_offset);
    calibData.appendFormat(CALIB_FMT_STRINGS[15], calib_data.relative_position_flag);
    calibData.appendFormat(CALIB_FMT_STRINGS[16], calib_data.relative_baseline_distance);
    calibData.appendFormat(CALIB_FMT_STRINGS[17], calib_data.main_sensor_mirror_flip_setting);
    calibData.appendFormat(CALIB_FMT_STRINGS[18], calib_data.aux_sensor_mirror_flip_setting);
    calibData.appendFormat(CALIB_FMT_STRINGS[19], calib_data.module_orientation_during_calibration);
    calibData.appendFormat(CALIB_FMT_STRINGS[20], calib_data.rotation_flag);
    calibData.appendFormat(CALIB_FMT_STRINGS[21],
            calib_data.main_cam_specific_calibration.normalized_focal_length);
    calibData.appendFormat(CALIB_FMT_STRINGS[22],
            calib_data.aux_cam_specific_calibration.normalized_focal_length);

    return calibData;
}

void QCameraBokeh::dumpInputParams(const char* file, String8 str, uint32_t idx)
{
    char filename[256];
    snprintf(filename, sizeof(filename),
            QCAMERA_DUMP_FRM_LOCATION"%s_%d.txt",file, idx);

    int file_fd = open(filename, O_RDWR | O_CREAT, 0777);
    if (file_fd > 0) {
        fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        write(file_fd, str.string(), str.size());
        close(file_fd);
    }
}
} // namespace qcamera
