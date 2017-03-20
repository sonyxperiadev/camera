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
#define LOG_TAG "QCameraHALPPManager"

#include "QCameraPprocManager.h"
#include "QCameraHALPP.h"
#include "QCameraDualFOVPP.h"
#include "QCameraBokeh.h"

using namespace android;

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraHALPPManager
 *
 * DESCRIPTION: constructor of QCameraHALPPManager.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraHALPPManager::QCameraHALPPManager(void *pUserData)
        : m_inputQ(releaseDataCb, this),
    //m_outgoingQ(releaseDataCb, this),
    m_pQCameraPostProc((QCameraPostProcessor*)pUserData),
    m_pPprocModule(NULL),
    m_pprocType(CAM_HAL_PP_TYPE_NONE),
    m_bInited(FALSE),
    m_bStarted(FALSE),
    m_halPPBufNotifyCB(NULL),
    m_halPPGetOutputCB(NULL)
{
}

/*===========================================================================
 * FUNCTION   : ~QCameraHALPPManager
 *
 * DESCRIPTION: destructor of QCameraHALPPManager.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraHALPPManager::~QCameraHALPPManager()
{
    if (m_pPprocModule) {
        delete m_pPprocModule;
        m_pPprocModule = NULL;
    }
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of HAL PProc manager
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPPManager::init(
        cam_hal_pp_type_t type,
        halPPBufNotify bufNotifyCb,
        halPPGetOutput getOutputCb,
        void *pStaticParam) {
    int32_t rc = NO_ERROR;
    LOGH("E halppType: %d", type);
    m_pprocType = type;
    m_halPPBufNotifyCB = bufNotifyCb;
    m_halPPGetOutputCB = getOutputCb;
    // Init the HAL Pproc module
    switch (m_pprocType) {
        case CAM_HAL_PP_TYPE_DUAL_FOV:
            m_pPprocModule = new QCameraDualFOVPP();
            break;
        case CAM_HAL_PP_TYPE_BOKEH:
            m_pPprocModule = new QCameraBokeh();
            break;
        case CAM_HAL_PP_TYPE_CLEARSIGHT:
            break;
        default:
            break;
    }
    if (m_pPprocModule != NULL) {
        rc = m_pPprocModule->init(
                QCameraHALPPManager::processHalPPDataCB,
                QCameraHALPPManager::getHalPPOutputBufferCB,
                this,
                pStaticParam);
        if (rc != NO_ERROR) {
            LOGE("HAL PP type %d init failed, rc = %d", m_pprocType, rc);
            delete m_pPprocModule;
            m_pPprocModule = NULL;
            return rc;
        }
        // Launch the Pproc thread
        m_pprocTh.launch(dataProcessRoutine, this);
        m_bInited = TRUE;
        LOGH("X");
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: deinitialization of HAL PProc manager
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPPManager::deinit() {
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (m_bInited == TRUE) {
        if (m_pPprocModule) {
            m_pPprocModule->deinit();
            delete m_pPprocModule;
            m_pPprocModule = NULL;
        }
        m_pprocType = CAM_HAL_PP_TYPE_NONE;
        m_halPPBufNotifyCB = NULL;
        m_halPPGetOutputCB = NULL;
        m_pprocTh.exit();
    }
    m_bInited = FALSE;
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start HAL PProc Manager. Data process thread will start.
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : HAL PP module will start
 *==========================================================================*/
int32_t QCameraHALPPManager::start()
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (m_bInited) {
        m_pprocTh.sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, TRUE, FALSE);
        m_bStarted = true;
    } else {
        m_bStarted = false;
        return -EACCES;
    }
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop HAL PProc Manager. Data process thread will stop.
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : HAL PP module will stop
 *==========================================================================*/
int32_t QCameraHALPPManager::stop()
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (m_bStarted) {
        Mutex::Autolock l(mLock);
        m_pprocTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, TRUE, TRUE);
        m_bStarted = false;
    }
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : feedInput
 *
 * DESCRIPTION: function to feed input data.
 *
 * PARAMETERS :
 *   @pInput  : ptr to input data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPPManager::feedInput(qcamera_hal_pp_data_t *pInput)
{
    int32_t rc = NO_ERROR;
    LOGD("E");
    if (!m_bStarted) {
        LOGE("X NOT SUPPORTED");
        return -EACCES;
    }
    // enqueue HAL PProc frame to input queue
        if (false == m_inputQ.enqueue((void *)pInput)) {
            LOGW("Input Q is not active!!!");
            releaseData(pInput);
            free(pInput);
    }
    // wake up data proc thread
    m_pprocTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : feedOutput
 *
 * DESCRIPTION: function to feed output buffer.
 *
 * PARAMETERS :
 *   @pOutputData  : ptr to output data buffer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPPManager::feedOutput(qcamera_hal_pp_data_t *pOutputData)
{
    int32_t rc = NO_ERROR;
    LOGD("E");
    if (!m_bStarted) {
        LOGE("X NOT SUPPORTED");
        return -EACCES;
    }
    m_pPprocModule->feedOutput(pOutputData);
    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseDataCb
 *
 * DESCRIPTION: callback function to release data
 *
 * PARAMETERS :
 *   @pData     : ptr to ongoing job data
 *   @pUserData : user data ptr (QCameraHALPPManager)
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPPManager::releaseDataCb(void *pData, void *pUserData)
{
    LOGH("E");
    if (pUserData != NULL && pData != NULL) {
        QCameraHALPPManager *pme = (QCameraHALPPManager *)pUserData;
        pme->releaseData((qcamera_hal_pp_data_t*)pData);
    }
    LOGH("X");
}

/*===========================================================================
 * FUNCTION   : releaseData
 *
 * DESCRIPTION: release buffer in qcamera_hal_pp_data_t
 *
 * PARAMETERS :
 *   @pData      : hal pp data
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPPManager::releaseData(qcamera_hal_pp_data_t *pData)
{
    if (pData) {
        if (pData->src_reproc_frame) {
            if (!pData->reproc_frame_release) {
                m_pQCameraPostProc->releaseSuperBuf(pData->src_reproc_frame);
            }
            free(pData->src_reproc_frame);
            pData->src_reproc_frame = NULL;
        }
        mm_camera_super_buf_t *frame = pData->frame;
        if (frame) {
            if (pData->halPPAllocatedBuf && pData->bufs) {
                free(pData->bufs);
            } else {
                m_pQCameraPostProc->releaseSuperBuf(frame);
            }
            free(frame);
            frame = NULL;
        }
        if (pData->snapshot_heap) {
            pData->snapshot_heap->deallocate();
            delete pData->snapshot_heap;
            pData->snapshot_heap = NULL;
        }
        if (pData->metadata_heap) {
            pData->metadata_heap->deallocate();
            delete pData->metadata_heap;
            pData->metadata_heap = NULL;
        }
        if (NULL != pData->src_reproc_bufs) {
            delete [] pData->src_reproc_bufs;
        }
        if ((pData->offline_reproc_buf != NULL)
                && (pData->offline_buffer)) {
            free(pData->offline_reproc_buf);
            pData->offline_reproc_buf = NULL;
            pData->offline_buffer = false;
        }
    }
}

/*===========================================================================
 * FUNCTION   : getSnapshotBuf
 *
 * DESCRIPTION: function to get snapshot buf def and the stream from frame
 * PARAMETERS :
 *   @pData           : input frame super buffer
 *   @pSnapshotStream : stream of snapshot that found
 * RETURN             : snapshot buf def
 *==========================================================================*/
mm_camera_buf_def_t* QCameraHALPPManager::getSnapshotBuf(qcamera_hal_pp_data_t* pData,
        QCameraStream* &pSnapshotStream)
{
    LOGH("E");
    mm_camera_buf_def_t *pBufDef = NULL;
    if (pData == NULL) {
        LOGE("Cannot find input frame super buffer");
        return pBufDef;
    }
    mm_camera_super_buf_t *pFrame = pData->frame;
    QCameraChannel *pChannel = m_pQCameraPostProc->getChannelByHandle(pFrame->ch_id);
    if (pChannel == NULL) {
        LOGE("Cannot find channel");
        return pBufDef;
    }
    // Search for input snapshot frame buf
    for (uint32_t i = 0; i < pFrame->num_bufs; i++) {
        pSnapshotStream = pChannel->getStreamByHandle(pFrame->bufs[i]->stream_id);
        if (pSnapshotStream != NULL) {
            if (pSnapshotStream->isTypeOf(CAM_STREAM_TYPE_SNAPSHOT) ||
                pSnapshotStream->isOrignalTypeOf(CAM_STREAM_TYPE_SNAPSHOT)) {
                    pBufDef = pFrame->bufs[i];
                    break;
            }
        }
    }
    LOGH("X");
    return pBufDef;
}

/*===========================================================================
 * FUNCTION   : getMetadataBuf
 *
 * DESCRIPTION: function to get metadata buf def and the stream from frame
 * PARAMETERS :
 *   @pData     : input frame super buffer
 *   @pMetadataStream : stream of metadata that found
 * RETURN     : metadata buf def
 *==========================================================================*/
mm_camera_buf_def_t* QCameraHALPPManager::getMetadataBuf(qcamera_hal_pp_data_t *pData,
        QCameraStream* &pMetadataStream)
{
    LOGH("E");
    mm_camera_buf_def_t *pBufDef = NULL;
    if ((pData == NULL) ||(pData->src_reproc_frame == NULL)) {
        LOGE("Cannot find input frame super buffer");
        return pBufDef;
    }
    mm_camera_super_buf_t* pFrame = pData->frame;
    QCameraChannel *pChannel =
            m_pQCameraPostProc->getChannelByHandle(pData->src_reproc_frame->ch_id);
    LOGD("src_reproc_frame num_bufs = %d", pFrame->num_bufs);
    if (pChannel == NULL) {
            LOGE("Cannot find src_reproc_frame channel");
            return pBufDef;
    }
    for (uint32_t i = 0;  i < pData->src_reproc_frame->num_bufs; i++) {
        pMetadataStream = pChannel->getStreamByHandle(pData->src_reproc_frame->bufs[i]->stream_id);
        if (pData->src_reproc_frame->bufs[i]->stream_type == CAM_STREAM_TYPE_METADATA) {
            pBufDef = pData->src_reproc_frame->bufs[i];
            LOGD("find metadata stream and buf from src_reproc_frame");
            break;
        }
    }
    if (pBufDef == NULL) {
        LOGD("frame num_bufs = %d", pFrame->num_bufs);
        pChannel = m_pQCameraPostProc->getChannelByHandle(pFrame->ch_id);
        if (pChannel == NULL) {
            LOGE("Cannot find frame channel");
            return pBufDef;
        }
        for (uint32_t i = 0; i < pFrame->num_bufs; i++) {
            pMetadataStream = pChannel->getStreamByHandle(pFrame->bufs[i]->stream_id);
            if (pMetadataStream != NULL) {
                LOGD("bufs[%d] stream_type = %d", i, pFrame->bufs[i]->stream_type);
                if (pFrame->bufs[i]->stream_type == CAM_STREAM_TYPE_METADATA) {
                    pBufDef = pFrame->bufs[i];
                    break;
                }
            }
        }
    }
    LOGH("X");
    return pBufDef;
}

/*===========================================================================
 * FUNCTION   : processHalPPDataCB
 *
 * DESCRIPTION: callback function to process frame after HAL PP block
 *
 * PARAMETERS :
 *   @pOutput     : output after HAL PP processed
 *   @pUserData   : user data ptr (QCameraHALPPManager)
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPPManager::processHalPPDataCB(qcamera_hal_pp_data_t *pOutput, void* pUserData)
{
    LOGH("E");
    QCameraHALPPManager *pme = (QCameraHALPPManager *)pUserData;
    pme->m_halPPBufNotifyCB(pOutput, pme->m_pQCameraPostProc);
    LOGH("X");
}

/*===========================================================================
 * FUNCTION   : getHalPPOutputBufferCB
 *
 * DESCRIPTION: callback function to request output buffer
 *
 * PARAMETERS :
 *   @frameIndex  : frameIndex needs to be appended in the output data
 *   @pUserData   : user data ptr (QCameraHALPPManager)
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPPManager::getHalPPOutputBufferCB(uint32_t frameIndex, void* pUserData)
{
    LOGH("E");
    QCameraHALPPManager *pme = (QCameraHALPPManager *)pUserData;
    pme->m_halPPGetOutputCB(frameIndex, pme->m_pQCameraPostProc);
    LOGH("X");
}

/*===========================================================================
 * FUNCTION   : dataProcessRoutine
 *
 * DESCRIPTION: data process routine that handles input data either from input
 *              PP Queue to do HAL post processing, or from outgoing PP Queue to
 *              send the final output frame back.
 *
 * PARAMETERS :
 *   @data    : user data ptr (QCameraHALPPManager)
 *
 * RETURN     : None
 *==========================================================================*/
void *QCameraHALPPManager::dataProcessRoutine(void *pData)
{
    int running = 1;
    int ret;
    uint8_t is_active = FALSE;
    QCameraHALPPManager *pme = (QCameraHALPPManager *)pData;
    QCameraCmdThread *cmdThread = &(pme->m_pprocTh);
    cmdThread->setName("CAM_HALPProc");
    LOGH("E");
    do {
        do {
            ret = cam_sem_wait(&cmdThread->cmd_sem);
            if (ret != 0 && errno != EINVAL) {
                LOGE("cam_sem_wait error (%s)",
                        strerror(errno));
                return NULL;
            }
        } while (ret != 0);
        // we got notified about new cmd avail in cmd queue
        camera_cmd_type_t cmd = cmdThread->getCmd();
        switch (cmd) {
        case CAMERA_CMD_TYPE_START_DATA_PROC:
            LOGH("start data proc");
            is_active = TRUE;
            pme->m_inputQ.init();
            //m_outgoingQ->init();
            if (pme->m_pPprocModule != NULL) {
                pme->m_pPprocModule->initQ();
            }
            // signal cmd is completed
            cam_sem_post(&cmdThread->sync_sem);
            break;
        case CAMERA_CMD_TYPE_STOP_DATA_PROC:
            {
                LOGH("stop data proc");
                is_active = FALSE;
                pme->m_inputQ.flush();
                //m_outgoingQ->flush();
                // flush m_halPP
                if (pme->m_pPprocModule != NULL) {
                    pme->m_pPprocModule->stop();
                    pme->m_pPprocModule->flushQ();
                }
                // signal cmd is completed
                cam_sem_post(&cmdThread->sync_sem);
            }
            break;
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                LOGH("Do next job, active is %d", is_active);
                if (is_active == TRUE) {
                    // Feed Input buffer to PP module
                    qcamera_hal_pp_data_t* inputJob =
                        (qcamera_hal_pp_data_t*)pme->m_inputQ.dequeue();
                    // Process HAL PP data if ready
                    if (pme->m_pPprocModule != NULL) {
                        if(inputJob != NULL) {
                            ret = pme->m_pPprocModule->feedInput(inputJob);
                            if (ret != NO_ERROR) {
                                LOGE("Error feeding input to HAL PP!!");
                            }
                        }
                        {
                            Mutex::Autolock l(pme->mLock);
                            pme->m_pPprocModule->process();
                        }
                    }
                }
            }
            break;
        case CAMERA_CMD_TYPE_EXIT:
            running = 0;
            break;
        default:
            break;
        }
    } while (running);
    LOGH("X");
    return NULL;
}

}; // namespace qcamera


