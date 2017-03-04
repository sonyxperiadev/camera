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

#ifndef __QCAMERA_BOKEH_H__
#define __QCAMERA_BOKEH_H__

// Camera dependencies
#include "QCameraHALPP.h"
#include "QCameraPostProc.h"

typedef struct  {
    cam_frame_size_t wide;
    cam_frame_size_t tele;
} bokeh_input_params_t;

typedef struct  {
    cam_frame_size_t out;
    uint32_t result;
} bokeh_output_params_t;


namespace qcamera {

typedef struct  {
    qcamera_hal_pp_data_t* wide_input;
    qcamera_hal_pp_data_t* tele_input;
    qcamera_hal_pp_data_t* tele_output;
} bokeh_data_t;

class QCameraBokeh : public QCameraHALPP
{
public:
    QCameraBokeh();
    ~QCameraBokeh();
    int32_t init(
            halPPBufNotify bufNotifyCb,
            halPPGetOutput getOutputCb,
            void *pUserData,
            void *pStaticParam);
    int32_t deinit();
    int32_t start();
    int32_t feedInput(qcamera_hal_pp_data_t *pInputData);
    int32_t feedOutput(qcamera_hal_pp_data_t *pOutputData);
    int32_t process();
protected:
    bool canProcess();
private:
    void getInputParams(bokeh_input_params_t& inParams);
    int32_t doBokehInit();
    int32_t doBokehProcess(
            const uint8_t* pWide,
            const uint8_t* pTele,
            bokeh_input_params_t inParams,
            uint8_t* pOut);
    void dumpYUVtoFile(
            const uint8_t* pBuf,
            cam_frame_len_offset_t offset,
            uint32_t idx,
            const char* name_prefix);
    void dumpInputParams(const bokeh_input_params_t& p);

private:
    void *m_dlHandle;
    const cam_capability_t *m_pCaps;
    bokeh_data_t mBokehData;
}; // QCameraBokeh class
}; // namespace qcamera

#endif /* __QCAMERA_BOKEH_H__ */

