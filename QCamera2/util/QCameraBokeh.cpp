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
                    pInputData->src_reproc_frame->camera_handle);
            uint32_t auxHandle = get_aux_camera_handle(
                    pInputData->src_reproc_frame->camera_handle);

            LOGH("mainHandle = 0x%x, auxHandle = 0x%x", mainHandle, auxHandle);
            if ((!mainHandle) && (!auxHandle)) {
                // Both main and aux handles are not available
                // Return from here
                return BAD_VALUE;
            }
            if (mainHandle && (mBokehData.wide_input == NULL)) {
                mBokehData.wide_input = pInputData;
                LOGH("Update wide input");
            }
            else if (auxHandle && (mBokehData.tele_input == NULL)) {
                mBokehData.tele_input = pInputData;
                LOGH("Update tele input");
            }
            // request output buffer only if both wide and tele input data are recieved
            if ((mBokehData.tele_input != NULL) && (mBokehData.wide_input != NULL)) {
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
    if (mBokehData.tele_output != NULL) {
        releaseData(pOutputData);
        LOGE("Error!! Output data is not empty");
        return BAD_VALUE;
    }
    rc = getOutputBuffer(mBokehData.tele_input, pOutputData);
    if (rc == NO_ERROR) {
        LOGH("filling Tele output %d", rc);
        mBokehData.tele_output = pOutputData;
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

    /* dump in/out frames */
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.camera.bokeh.dumpimg", prop, "0");
    int dumpimg = atoi(prop);

    LOGH("E");

    // Start the blending process when it is ready
    if (canProcess()) {
        LOGH("Starting Bokeh process");
        QCameraStream* pTeleStream = NULL;
        QCameraStream* pWideStream = NULL;

        mm_camera_buf_def_t *pTeleSnap = getSnapshotBuf(mBokehData.tele_input, pTeleStream);
        mm_camera_buf_def_t *pWideSnap = getSnapshotBuf(mBokehData.wide_input, pWideStream);

        mm_camera_super_buf_t *pOutputSuperBuf = mBokehData.tele_output->frame;
        mm_camera_buf_def_t *pOutputBuf = pOutputSuperBuf->bufs[0];

        // Use offset info from reproc stream
        cam_frame_len_offset_t frm_offset;
        memset(&frm_offset, 0, sizeof(frm_offset));
        pTeleStream->getFrameOffset(frm_offset);
        LOGH("Stream type:%d, stride:%d, scanline:%d, frame len:%d",
                pTeleStream->getMyType(),
                frm_offset.mp[0].stride, frm_offset.mp[0].scanline,
                frm_offset.frame_len);

        //Get input and output parameter
        bokeh_input_params_t inParams;
        getInputParams(inParams);

        LOGH("doing Bokeh process!!!");
        doBokehProcess(
                (const uint8_t *)pTeleSnap->buffer,
                (const uint8_t *)pWideSnap->buffer,
                inParams,
                (uint8_t *)pOutputBuf->buffer);

        if (dumpimg) {
            dumpYUVtoFile((uint8_t *)pTeleSnap->buffer, frm_offset,
                    pTeleSnap->frame_idx, "Tele");
            dumpYUVtoFile((uint8_t *)pWideSnap->buffer,  frm_offset,
                    pWideSnap->frame_idx,  "Wide");
            dumpYUVtoFile((uint8_t *)pOutputBuf->buffer, frm_offset,
                    pTeleSnap->frame_idx, "TeleBokeh");
        }

        // Invalidate input buffer
        QCameraMemory *pMem = NULL;
        pMem = (QCameraMemory *)pTeleSnap->mem_info;
        pMem->invalidateCache(pTeleSnap->buf_idx);
        pMem = (QCameraMemory *)pWideSnap->mem_info;
        pMem->invalidateCache(pWideSnap->buf_idx);
        // Clean and invalidate output buffer
        mBokehData.tele_output->snapshot_heap->cleanInvalidateCache(0);

        // Callback Manager to notify output buffer and return input buffers
        LOGH("notifying HAL PP tele output CB");
        m_halPPBufNotifyCB(mBokehData.tele_output, m_pHalPPMgr);
        LOGH("CB for tele input");
        m_halPPBufNotifyCB(mBokehData.tele_input, m_pHalPPMgr);
        LOGH("CB for wide input");
        m_halPPBufNotifyCB(mBokehData.wide_input, m_pHalPPMgr);
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
    LOGH("E");
    bool ready = false;
    //Check if we have all input and output buffers
    if (mBokehData.wide_input && mBokehData.tele_input && mBokehData.tele_output) {
        ready = true;
        LOGH("ready: %d", ready);
    }
    LOGH("X");
    return ready;
}

/*===========================================================================
 * FUNCTION   : getInputParams
 *
 * DESCRIPTION: Helper function to get input params from input metadata
 *==========================================================================*/
void QCameraBokeh::getInputParams(bokeh_input_params_t& inParams)
{
    LOGH("E");
    memset(&inParams, 0, sizeof(bokeh_input_params_t));
    QCameraStream* pTeleStream = NULL;
    QCameraStream* pWideStream = NULL;
    //QCameraStream* pTeleMetaStream  = NULL;
    //QCameraStream* pWideMetaStream  = NULL;

    mm_camera_buf_def_t *pTeleSnap = getSnapshotBuf(mBokehData.tele_input, pTeleStream);
    mm_camera_buf_def_t *pWideSnap = getSnapshotBuf(mBokehData.wide_input, pWideStream);
    if (!pTeleSnap || !pWideSnap) {
        LOGH("NULL pointer pTeleSnap: %p, pWideSnap: %p", pTeleSnap, pWideSnap);
        return;
    }

    //mm_camera_buf_def_t *pTeleMeta = getMetadataBuf(mBokehData.tele_input, pTeleMetaStream);
    //mm_camera_buf_def_t *pWideMeta = getMetadataBuf(mBokehData.wide_input, pWideMetaStream);
    //metadata_buffer_t *pTeleMetaBuf = (metadata_buffer_t *)pTeleMeta->buffer;
    //metadata_buffer_t *pWideMetaBuf = (metadata_buffer_t *)pWideMeta->buffer;

    // Wide frame size
    cam_frame_len_offset_t offset;
    pWideStream->getFrameOffset(offset);
    inParams.wide.width     = offset.mp[0].width;
    inParams.wide.height    = offset.mp[0].height;
    inParams.wide.stride    = offset.mp[0].stride;
    inParams.wide.scanline  = offset.mp[0].scanline;
    inParams.wide.frame_len = offset.frame_len;
    LOGH("Wide width: %d height: %d stride:%d, scanline:%d frame_len: %d",
            inParams.wide.width, inParams.wide.height,
            inParams.wide.stride, inParams.wide.scanline,
            inParams.wide.frame_len);

    // Tele frame size
    pTeleStream->getFrameOffset(offset);
    inParams.tele.width     = offset.mp[0].width;
    inParams.tele.height    = offset.mp[0].height;
    inParams.tele.stride    = offset.mp[0].stride;
    inParams.tele.scanline  = offset.mp[0].scanline;
    inParams.tele.frame_len = offset.frame_len;

    LOGH("Tele width: %d height: %d stride:%d, scanline:%d",
            inParams.tele.width, inParams.tele.height,
            inParams.tele.stride, inParams.tele.scanline,
            inParams.tele.frame_len);

    dumpInputParams(inParams);

    LOGH("X");
}


int32_t QCameraBokeh::doBokehInit()
{
    LOGH("E");
    int rc = NO_ERROR;
    LOGH("E");
    return rc;
}

int32_t QCameraBokeh::doBokehProcess(
        const uint8_t* pWide,
        const uint8_t* pTele,
        bokeh_input_params_t inParams,
        uint8_t* pOut)
{
    LOGH("E pWide:%p, pTele:%p", pWide, pTele);

    // Copy half of wide and half of Tele if both sizes are same
    if ((inParams.tele.stride == inParams.wide.stride) &&
            (inParams.tele.stride == inParams.wide.stride)) {
        // Y
        memcpy(pOut, pWide, inParams.wide.stride * inParams.wide.scanline / 2);
        memcpy(pOut  + inParams.wide.stride * inParams.wide.scanline / 2,
               pTele + inParams.wide.stride * inParams.wide.scanline / 2,
               inParams.wide.stride * inParams.wide.scanline / 2);

        // UV
        uint32_t uv_offset = inParams.wide.stride * inParams.wide.scanline;
        memcpy(pOut  + uv_offset,
               pWide + uv_offset,
               inParams.wide.stride * (inParams.wide.scanline / 2) / 2);
        memcpy(pOut  + uv_offset + inParams.wide.stride * (inParams.wide.scanline / 2) / 2,
               pTele + uv_offset + inParams.wide.stride * (inParams.wide.scanline / 2) / 2,
               inParams.wide.stride * (inParams.wide.scanline / 2) / 2);
    } else {
        // Copy Tele if both sizes are not same
        // Y
        memcpy(pOut, pTele, inParams.tele.stride * inParams.tele.scanline);
        // UV
        uint32_t uv_offset = inParams.tele.stride * inParams.tele.scanline;
        memcpy(pOut  + uv_offset, pTele + uv_offset,
                inParams.tele.stride * (inParams.tele.scanline / 2));
    }

    LOGH("X.");
    return NO_ERROR;
}

void QCameraBokeh::dumpYUVtoFile(
        const uint8_t* pBuf,
        cam_frame_len_offset_t offset,
        uint32_t idx,
        const char* name_prefix)
{
    LOGH("E.");
    char filename[256];

    snprintf(filename, sizeof(filename), QCAMERA_DUMP_FRM_LOCATION"%s_%dx%d_%d.yuv",
                name_prefix, offset.mp[0].stride, offset.mp[0].scanline, idx);

    QCameraHALPP::dumpYUVtoFile(pBuf,(const char*)filename, offset.frame_len);

    LOGH("X.");
}

void QCameraBokeh::dumpInputParams(const bokeh_input_params_t& p)
{
    LOGH("E");

    const cam_frame_size_t* s = NULL;

    s = &p.wide;
    LOGH("wide frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    s = &p.tele;
    LOGH("wide frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    LOGH("X");
}

} // namespace qcamera
