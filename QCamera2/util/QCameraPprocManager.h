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

#ifndef __QCAMERA_HAL_PPROC_MANAGER_H__
#define __QCAMERA_HAL_PPROC_MANAGER_H__

// Camera dependencies
#include "QCameraQueue.h"
#include "QCamera2HWI.h"
#include "QCameraPostProc.h"

using namespace android;
namespace qcamera {

/** halPPBufNotify: function definition for frame notify
*   handling
*    @pOutput  : received qcamera_hal_pp_data_t data
*    @pUserData: user data pointer
**/
typedef void (*halPPBufNotify) (qcamera_hal_pp_data_t *pOutput,
                                        void *pUserData);

/** halPPGetOutput: function definition for get output buffer
*    @frameIndex: output frame index should match input frame index
*    @pUserData: user data pointer
**/
typedef void (*halPPGetOutput) (uint32_t frameIndex, void *pUserData);

class QCameraHALPP;

class QCameraHALPPManager
{
public:
    QCameraHALPPManager(void *pUserData);
    ~QCameraHALPPManager();
    int32_t init(cam_hal_pp_type_t type, halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb, void *pStaticParam);
    int32_t deinit();
    int32_t start();
    int32_t stop();
    int32_t feedInput(qcamera_hal_pp_data_t *pInput);
    int32_t feedOutput(qcamera_hal_pp_data_t *pOutputData);
    void releaseData(qcamera_hal_pp_data_t *pData);
    mm_camera_buf_def_t* getSnapshotBuf(qcamera_hal_pp_data_t* pData,
            QCameraStream* &pSnapshotStream);
    mm_camera_buf_def_t* getMetadataBuf(qcamera_hal_pp_data_t* pData,
            QCameraStream* &pMetadataStream);
    cam_hal_pp_type_t getPprocType() { return m_pprocType;};

private:
    static void *dataProcessRoutine(void *pData);
    Mutex  mLock;

protected:
    static void releaseDataCb(void *pData, void *pUserData);
    static void processHalPPDataCB(qcamera_hal_pp_data_t *pOutput, void* pUserData);
    static void getHalPPOutputBufferCB(uint32_t frameIndex, void* pUserData);

protected:
    QCameraQueue m_inputQ;
    QCameraPostProcessor *m_pQCameraPostProc;
    QCameraHALPP *m_pPprocModule;
    cam_hal_pp_type_t m_pprocType;
    QCameraCmdThread m_pprocTh;      // thread for data processing
    bool m_bInited;
    bool m_bStarted;

    halPPBufNotify m_halPPBufNotifyCB;
    halPPGetOutput m_halPPGetOutputCB;
}; // QCameraHALPPManager class
}; // namespace qcamera

#endif /* __QCAMERA_HAL_PPROC_MANAGER_H__ */


