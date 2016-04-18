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

#define LOG_TAG "QCameraCommon"

// System dependencies
#include <utils/Errors.h>
#include <stdlib.h>
#include <string.h>
#include <utils/Log.h>

// Camera dependencies
#include "QCameraCommon.h"

using namespace android;

namespace qcamera {

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*===========================================================================
 * FUNCTION   : QCameraCommon
 *
 * DESCRIPTION: default constructor of QCameraCommon
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraCommon::QCameraCommon() :
    m_pCapability(NULL)
{
}

/*===========================================================================
 * FUNCTION   : ~QCameraCommon
 *
 * DESCRIPTION: destructor of QCameraCommon
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraCommon::~QCameraCommon()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: Init function for QCameraCommon
 *
 * PARAMETERS :
 *   @pCapability : Capabilities
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCommon::init(cam_capability_t *pCapability)
{
    m_pCapability = pCapability;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getAnalysisInfo
 *
 * DESCRIPTION: Get the Analysis information based on
 *     current mode and feature mask
 *
 * PARAMETERS :
 *   @fdVideoEnabled : Whether fdVideo enabled currently
 *   @hal3           : Whether hal3 or hal1
 *   @featureMask    : Feature mask
 *   @analysis_info  : Analysis info to be filled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCommon::getAnalysisInfo(
        bool fdVideoEnabled,
        bool hal3,
        uint32_t featureMask,
        cam_analysis_info_t *pAnalysisInfo)
{
    if ((fdVideoEnabled == TRUE) && (hal3 == FALSE) &&
            (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_VIDEO].hw_analysis_supported) &&
            (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_VIDEO].valid)) {
        *pAnalysisInfo =
                m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_VIDEO];
    } else if (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_STILL].valid) {
        *pAnalysisInfo =
                m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_STILL];
        if (hal3 == TRUE) {
            pAnalysisInfo->analysis_max_res = pAnalysisInfo->analysis_recommended_res;
        }
    }

    return 0;
}

}; // namespace qcamera
