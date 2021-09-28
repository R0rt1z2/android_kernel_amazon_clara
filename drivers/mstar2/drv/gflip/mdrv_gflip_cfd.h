/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _MDRV_GFLIP_CFD_H
#define _MDRV_GFLIP_CFD_H

//CFD Version define
#define CFD_CONTROL_POINT_ST_VERSION (1)
#define STU_CFDAPI_GOP_PRESDRIP_VERSION (1)

typedef struct _STU_CFDAPI_UI_CONTROL
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_HDMI_OSD_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFDAPI_UI_CONTROL)

    //1x = 50
    //range = [0-100]
    MS_U16 u16Hue;

    //1x = 128
    //range = [0-255]
    MS_U16 u16Saturation;

    //1x = 1024
    //range = [0-2047]
    MS_U16 u16Contrast;

    MS_U8  u8ColorCorrection_En;

    //1x = 1024
    //rnage = [-2048-2047]
    MS_S32 s32ColorCorrectionMatrix[3][3];

    //1x = 1024
    //range = [0-2047]
    MS_U16 u16Brightness;

    //1x = 1024
    //range = [0-2048]
    //u16RGBGGain[0] = Rgain
    //u16RGBGGain[1] = Ggain
    //u16RGBGGain[2] = Bgain
    MS_U16 u16RGBGGain[3];

    //0:off
    //1:on
    //default on , not in the document
    MS_U8  u8OSD_UI_En;

    //Mode 0: update matrix by OSD and color format driver
    //Mode 1: only update matrix by OSD controls
    //for mode1 : the configures of matrix keep the same as the values by calling CFD last time
    MS_U8  u8OSD_UI_Mode;

    //0:auto depends on STB rule
    //1:always do HDR2SDR for HDR input
    //2:always not do HDR2SDR for HDR input
    MS_U8  u8HDR_UI_H2SMode;
} STU_CFDAPI_UI_CONTROL;

typedef struct _STU_CFDAPI_DEBUG
{
    MS_U32 u32Version;
    MS_U16 u16Length;

    MS_U8  ShowALLInputInCFDEn;

}STU_CFDAPI_DEBUG;

typedef struct _STU_CFDAPI_GOP_FORMAT
{
    MS_U32 u32Version;
    MS_U16 u16Length;

    //bit[0]:
    //1: GOP use premultiplied Alpha

    //bit[1]: IsAlphaForGOPFlag
    //1: alpha value is for GOP
    MS_U8  u8GOP_AlphaFormat;

}STU_CFDAPI_GOP_FORMAT;

typedef struct _STU_CFDAPI_GOP_PRESDRIP
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_MAIN_CONTROL_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFDAPI_Kano_SDRIP)

    //IP2 CSC
    MS_U8 u8CSC_Mode;
    MS_U8 u8CSC_Ratio1;
    MS_U8 u8CSC_Manual_Vars_en;
    MS_U8 u8CSC_MC;

} STU_CFDAPI_GOP_PRESDRIP;

typedef struct _STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP
{

    STU_CFDAPI_GOP_PRESDRIP  stu_GOP_PRESDRIP_Param;

} STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP;

typedef struct _STU_CFDAPI_HW_IPS_GOP
{

    //u8HWGroup is for GOP group ID, starts from 0
    MS_U8 u8HWGroup;

    STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP *pstu_PRESDRIP_Input;

} STU_CFDAPI_HW_IPS_GOP;

typedef struct _STU_CFD_COLORIMETRY
{
    //order R->G->B
    MS_U16 u16Display_Primaries_x[3];      //data *0.00002 0xC350 = 1
    MS_U16 u16Display_Primaries_y[3];      //data *0.00002 0xC350 = 1
    MS_U16 u16White_point_x;               //data *0.00002 0xC350 = 1
    MS_U16 u16White_point_y;               //data *0.00002 0xC350 = 1

} STU_CFD_COLORIMETRY;

typedef struct _STU_CFDAPI_PANEL_FORMAT
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_HDMI_PANEL_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFDAPI_PANEL_FORMAT)

    MS_U16 u16Panel_Med_Luminance;          //data * 1 nits
    MS_U16 u16Panel_Max_Luminance;          //data * 1 nits
    MS_U16 u16Panel_Min_Luminance;          //data * 0.0001 nits

    //order R->G->B
    STU_CFD_COLORIMETRY stu_Cfd_Panel_ColorMetry;

} STU_CFDAPI_PANEL_FORMAT;

typedef struct _STU_CFDAPI_UI_CONTROL_TESTING
{

    MS_U32 u32TestCases;

} STU_CFDAPI_UI_CONTROL_TESTING;

typedef struct _STU_CFDAPI_MAIN_CONTROL_GOP_TESTING
{

    MS_U32 u32TestCases;

} STU_CFDAPI_MAIN_CONTROL_GOP_TESTING;

typedef struct _STU_CFDAPI_MAIN_CONTROL_GOP
{

    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_MAIN_CONTROL_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFD_MAIN_CONTROL)

    //E_CFD_MC_SOURCE
    //specify which input source
    MS_U8 u8Input_Source;

    //E_CFD_CFIO
    MS_U8 u8Input_Format;

    //E_CFD_MC_FORMAT
    //specify RGB/YUV format of the input of the first HDR/SDR IP
    //E_CFD_MC_FORMAT_RGB       = 0x00,
    //E_CFD_MC_FORMAT_YUV422    = 0x01,
    //E_CFD_MC_FORMAT_YUV444    = 0x02,
    //E_CFD_MC_FORMAT_YUV420    = 0x03,
    MS_U8 u8Input_DataFormat;

    //limit/full
    //assign with E_CFD_CFIO_RANGE
    //0:limit 1:full
    MS_U8 u8Input_IsFullRange;

    //SDR/HDR
    //E_CFIO_HDR_STATUS
    //0:SDR
    //1:HDR1
    //2:HDR2
    MS_U8 u8Input_HDRMode;

    //assign by E_CFD_CFIO_CP
    MS_U8 u8Input_ext_Colour_primaries;

    //assign by E_CFD_CFIO_TR
    MS_U8 u8Input_ext_Transfer_Characteristics;

    //assign by E_CFD_CFIO_MC
    MS_U8 u8Input_ext_Matrix_Coeffs;

    //used this gamut when u8Input_ext_Colour_primaries is E_CFD_CFIO_CP_SOURCE
    STU_CFD_COLORIMETRY stu_Cfd_source_ColorMetry;

    //specify RGB/YUV format of the output of the last HDR/SDR IP
    //E_CFD_MC_SOURCE
    MS_U8 u8Output_Source;

    //E_CFD_CFIO
    MS_U8 u8Output_Format;

    //E_CFD_MC_FORMAT
    MS_U8 u8Output_DataFormat;

    //0:limit 1:full
    MS_U8 u8Output_IsFullRange;

    //E_CFIO_HDR_STATUS
    //0:SDR
    //1:HDR1
    //2:HDR2
    MS_U8 u8Output_HDRMode;

    //assign by E_CFD_CFIO_CP
    MS_U8 u8Output_ext_Colour_primaries;

    //assign by E_CFD_CFIO_TR
    MS_U8 u8Output_ext_Transfer_Characteristics;

    //assign by E_CFD_CFIO_MC
    MS_U8 u8Output_ext_Matrix_Coeffs;

    //used this gamut when u8Output_ext_Colour_primaries is E_CFD_CFIO_CP_SOURCE
    STU_CFD_COLORIMETRY stu_Cfd_target_ColorMetry;

    //STU_CFDAPI_MAIN_CONTROL_GOP_TESTING stu_Cfd_main_control_testing;


} STU_CFDAPI_MAIN_CONTROL_GOP;

typedef struct _STU_CFDAPI_TOP_CONTROL_GOP
{
    //share with different HW
    MS_U32 u32Version;
    MS_U16 u16Length;
    STU_CFDAPI_MAIN_CONTROL_GOP      *pstu_Main_Control;
    //STU_CFDAPI_MM_PARSER           *pstu_MM_Param;
    //STU_CFDAPI_HDMI_EDID_PARSER    *pstu_HDMI_EDID_Param;
    STU_CFDAPI_UI_CONTROL            *pstu_UI_Param;
    STU_CFDAPI_PANEL_FORMAT          *pstu_Panel_Param;
    STU_CFDAPI_HW_IPS_GOP            *pstu_HW_IP_Param;
    STU_CFDAPI_GOP_FORMAT            *pstu_GOP_Param;

    STU_CFDAPI_DEBUG                 *pstu_Debug_Param;

} STU_CFDAPI_TOP_CONTROL_GOP;

typedef struct _STU_CFD_MS_ALG_COLOR_FORMAT_LITE
{
    //E_CFD_CFIO
    MS_U8 u8Input_Format;

    //E_CFD_MC_FORMAT
    MS_U8 u8Input_DataFormat;

    //limit/full
    //assign with E_CFD_CFIO_RANGE
    //0:limit 1:full
    MS_U8 u8Input_IsFullRange;

    //SDR/HDR
    //0:SDR
    //1:HDR1
    //2:HDR2
    MS_U8 u8Input_HDRMode;

    //follow E_CFD_CFIO_GAMUTORDER_IDX
    //use for non-panel output
    //MS_U8 u8Input_GamutOrderIdx;

    //E_CFD_CFIO_RGB
    //E_CFD_CFIO_YUV
    //E_CFD_CFIO
    MS_U8 u8Output_Format;

    MS_U8 u8Output_DataFormat;

    //MS_U8 u8Output_GamutOrderIdx;
    //follow E_CFD_CFIO_GAMUTORDER_IDX
    //use for non-panel output

    MS_U8 u8Output_IsFullRange;

    //SDR/HDR
    //0:SDR
    //1:HDR1
    //2:HDR2
    MS_U8 u8Output_HDRMode;

    //Temp_Format[0] : output of IP2 CSC, input of HDR IP
    //Temp_Format[1] : output of HDR IP, input of SDR IP
    MS_U8 u8Temp_Format[2];

    //E_CFD_MC_FORMAT
    MS_U8 u8Temp_DataFormat[2];

    //E_CFD_CFIO_RANGE
    MS_U8 u8Temp_IsFullRange[2];

    //E_CFIO_HDR_STATUS
    MS_U8 u8Temp_HDRMode[2];

    //MS_U8 u8Temp_GamutOrderIdx[2];

    //redefinition
    //item 0-10 are the same as table E.3 in HEVC spec
    MS_U8 u8InputColorPriamries;
    MS_U8 u8OutputColorPriamries;
    MS_U8 u8TempColorPriamries[2];
    //0:Reserverd
    //1:BT. 709/sRGB/sYCC
    //2:unspecified
    //3:Reserverd
    //4:BT. 470-6
    //5:BT. 601_625/PAL/SECAM
    //6:BT. 601_525/NTSC/SMPTE_170M
    //7:SMPTE_240M
    //8:Generic film
    //9:BT. 2020
    //10:CIEXYZ
    //11:AdobeRGB
    //255:undefined

    //redefinition
    //item 0-17 are the same as table E.4 in HEVC spec
    MS_U8 u8InputTransferCharacterstics;
    MS_U8 u8OutputTransferCharacterstics;
    MS_U8 u8TempTransferCharacterstics[2];
    //0 Reserverd
    //1 BT. 709
    //2 unspecified
    //3 Reserverd
    //4 Assume display gamma 2.2
    //5 Assume display gamma 2.8
    //6 BT. 601_525/BT. 601_525
    //7 SMPTE_240M
    //8 linear
    //9 Logarithmic (100:1 range)
    //10    Logarithmic (100*sqrt(10):1 range)
    //11    xvYCC
    //12    BT. 1361 extend color gamut system
    //13    sRGB/sYCC
    //14    BT. 2020
    //15    BT. 2020
    //16    SMPTE ST2084 for 10.12.14.16-bit systetm
    //17    SMPTE ST428-1
    //18   AdobeRGB
    //255  undefined

    //the same the same as table E.5 in HEVC spec
    MS_U8 u8InputMatrixCoefficients;
    MS_U8 u8OutputMatrixCoefficients;
    MS_U8 u8TempMatrixCoefficients[2];
    //0 Identity
    //1 BT. 709/xvYCC709
    //2 unspecified
    //3 Reserverd
    //4 USFCCT 47
    //5 BT. 601_625/PAL/SECAM/xvYCC601/sYCC
    //6 BT. 601_525/NTSC/SMPTE_170M
    //7 SMPTE_240M
    //8 YCgCo
    //9 BT. 2020NCL(non-constant luminance)
    //10    BT. 2020CL(constant luminance)
    //255:Reserverd
    STU_CFD_COLORIMETRY stu_Cfd_ColorMetry[4];


    //temp variable for debug
    MS_U16 u16_check_status;

    MS_U8 u8DoTMO_Flag;
    MS_U8 u8DoGamutMapping_Flag;
    MS_U8 u8DoDLC_Flag;
    MS_U8 u8DoBT2020CLP_Flag;

    //0 or 1
    //0: u8HWGroup is for mainsub
    //1: u8HWGroup is for GOP groups
    MS_U8 u8HWGroupMode;
    MS_U8 u8HWGroup;

} STU_CFD_MS_ALG_COLOR_FORMAT_LITE;

typedef struct _STU_CFDAPI_PREPROCESS_STATUS_LITE
{
    MS_U32 u32Version;
    MS_U16 u16Length;

    STU_CFD_MS_ALG_COLOR_FORMAT_LITE stu_Cfd_color_format;

}STU_CFDAPI_PREPROCESS_STATUS_LITE;

typedef struct _STU_CFDAPI_GOP_PREPROCESS_OUT
{
    MS_U32 u32Version;
    MS_U16 u16Length;

    STU_CFDAPI_PREPROCESS_STATUS_LITE    stu_Cfd_preprocess_status;

}STU_CFDAPI_GOP_PREPROCESS_OUT;

typedef struct _STU_CFD_GENERAL_CONTROL_GOP
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_MAIN_CONTROL_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFD_MAIN_CONTROL)

    //MS_U8 u8InputSource;
    //MS_U8 u8MainSubMode;
    //MS_U8 u8IsRGBBypass;

    MS_U8 u8HWGroupMode;
    MS_U8 u8HWGroup;

    //E_CFD_CFIO_RANGE_LIMIT
    //E_CFD_CFIO_RANGE_FULL
    //MS_U8 u8DoPathFullRange;
    STU_CFDAPI_DEBUG                 stu_Debug_Param;

} STU_CFD_GENERAL_CONTROL_GOP;

typedef struct _STU_CFD_CONTROL_POINT
{
    MS_U32 u32Version;   ///<Version of current structure. Please always set to "CFD_CONTROL_POINT_ST_VERSION" as input
    MS_U16 u16Length;    ///<Length of this structure, u16Length=sizeof(STU_CFD_CONTROL_POINT)
    MS_U8 u8MainSubMode;
    MS_U8 u8Format;
    MS_U8 u8DataFormat;
    MS_U8 u8IsFullRange;
    MS_U8 u8HDRMode;
    MS_U8 u8SDRIPMode;
    MS_U8 u8HDRIPMode;
    MS_U8 u8GamutOrderIdx;
    MS_U8 u8ColorPriamries;
    MS_U8 u8TransferCharacterstics;
    MS_U8 u8MatrixCoefficients;

    MS_U16 u16BlackLevelY;
    MS_U16 u16BlackLevelC;
    MS_U16 u16WhiteLevelY;
    MS_U16 u16WhiteLevelC;

    /// HDR10plus
    MS_U32 u16PanelMaxLuminance;
    MS_U32 u32NormGain;

    // new part
    // for CFD control
    // 0: call Mapi_Cfd_Preprocessing and Mapi_Cfd_Decision
    // 1: call Mapi_Cfd_Preprocessing only
    MS_U8 u8Cfd_process_mode;

    // Process Mode
    // 0: color format driver on - auto
    // CFD process depends on the input of CFD
    // 1: shows SDR UI, UI 709, no video
    MS_U8 u8PredefinedProcess;

    // report Process
    // bit0 DoDLCflag
    // bit1 DoGamutMappingflag
    // bit2 DoTMOflag
    MS_U8 u8Process_Status;
    MS_U8 u8Process_Status2;
    MS_U8 u8Process_Status3;

    MS_U16 u16_check_status;
    STU_CFD_COLORIMETRY stu_Source_Param_Colorimetry;
    STU_CFD_COLORIMETRY stu_Panel_Param_Colorimetry;
} STU_CFD_CONTROL_POINT;

typedef struct
{
    STU_CFD_GENERAL_CONTROL_GOP          *pstu_Cfd_General_Control;
    STU_CFD_CONTROL_POINT                 *pstu_Cfd_Control_Point_Front;
    STU_CFD_CONTROL_POINT                 *pstu_Cfd_Control_Point_End;
    STU_CFD_MS_ALG_INTERFACE_GOP_PRESDRIP *pstu_SDRIP_Param;
    STU_CFDAPI_UI_CONTROL                 *pstu_UI_control;

} ST_CFD_GOP_PRESDR_INPUT;

typedef struct _STU_Register_Table
{
    MS_U32 u32Depth;
    MS_U32 *pu32Address;
    MS_U16 *pu16Value;
    MS_U16 *pu16Mask;
    MS_U16 *pu16Client;
} STU_Register_Table;

typedef struct _STU_Autodownload_Table
{
    MS_U16 u16client;
    MS_U8 *pu8Data ;
    MS_U32 u32Size;
} STU_Autodownload_Table;

typedef struct _ST_STATUS
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    MS_U16 u16UpdateStatus; // Check whitch IPs need to update
}ST_STATUS;

typedef struct
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    STU_Register_Table stRegTable;
    STU_Autodownload_Table stAdlTable;
    ST_STATUS stStatus;
}ST_CFD_CSC_HW_OUTPUT;

// Coloect all output
typedef struct
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    STU_Register_Table stRegTable;
    STU_Autodownload_Table stAdlTable;
    ST_STATUS stStatus;
}ST_CFD_GENERAL_HW_OUTPUT;

typedef struct
{
    MS_U32 u32Version;
    MS_U16 u16Length;
    ST_CFD_CSC_HW_OUTPUT     stu_cfd_GOP_PreSDR_CSC_hw_output;
    ST_CFD_GENERAL_HW_OUTPUT stu_cfd_GOP_PreSDR_RGBOffset_hw_output;

} ST_CFD_GOP_PreSDR_HW_OUTPUT;

typedef struct _STU_CFDAPI_TOP_CONTROL_GOP_TESTING
{
    //share with different HW
    MS_U32 u32Version;
    MS_U16 u16Length;

    MS_U8  u8TestEn;

    STU_CFDAPI_MAIN_CONTROL_GOP_TESTING   stu_Main_Control;

    STU_CFDAPI_UI_CONTROL_TESTING         stu_UI_Param;
    //STU_CFDAPI_PANEL_FORMAT_TESTING       stu_Panel_Param;
    //STU_CFDAPI_HW_IPS_GOP_TESTING         stu_HW_IP_Param;
    //STU_CFDAPI_GOP_FORMAT_TESTING         stu_GOP_Param;

} STU_CFDAPI_TOP_CONTROL_GOP_TESTING;

typedef enum _E_CFD_MC_ERR//enum for u8HDR2SDR_Mode/u8SDR_Mode
{
    //Main control starts from 0x0000
    //if error happens, see Mapi_Cfd_inter_Main_Control_Param_Check()
    E_CFD_MC_ERR_NOERR      = 0x0000, //process is ok
    E_CFD_MC_ERR_INPUT_SOURCE    = 0x0001, //input source is over defined range, please check. force input_source to E_CFD_MC_SOURCE_HDMI now!!
    E_CFD_MC_ERR_INPUT_ANALOGIDX = 0x0002, //input analog idx is over defined range, please check. force input analog idx to E_CFD_INPUT_ANALOG_RF_NTSC_44 now!!
    E_CFD_MC_ERR_INPUT_FORMAT = 0x0003, //input format is over defined range, please check. force input format to E_CFD_CFIO_YUV_BT709 now!!

    E_CFD_MC_ERR_INPUT_DATAFORMAT = 0x0004, //input data format is over defined range, please check. force input data format to E_CFD_MC_FORMAT_YUV422 now!!
    E_CFD_MC_ERR_INPUT_ISFULLRANGE = 0x0005, //input data format is over defined range, please check. force MainControl u8Input_IsFullRange to E_CFD_CFIO_RANGE_LIMIT!!

    E_CFD_MC_ERR_INPUT_HDRMODE = 0x0006, //input HDR mode is over defined range, please check. force MainControl u8Input_HDRMode to E_CFIO_MODE_SDR!!
    E_CFD_MC_ERR_INPUT_ISRGBBYPASS = 0X0007,

    E_CFD_MC_ERR_INPUT_SDRIPMODE = 0x0008,
    E_CFD_MC_ERR_INPUT_HDRIPMODE = 0x0009,

    E_CFD_MC_ERR_INPUT_Mid_Format_Mode = 0x000a,
    E_CFD_MC_ERR_INPUT_Mid_Format       = 0x000b,
    E_CFD_MC_ERR_INPUT_Mid_DataFormat   = 0x000c,
    E_CFD_MC_ERR_INPUT_Mid_IsFullRange  = 0x000d,
    E_CFD_MC_ERR_INPUT_Mid_HDRMode      = 0x000e,
    E_CFD_MC_ERR_INPUT_Mid_Colour_primaries = 0x000f,
    E_CFD_MC_ERR_INPUT_Mid_Transfer_Characteristics = 0x0010,
    E_CFD_MC_ERR_INPUT_Mid_Matrix_Coeffs = 0x0011,

    E_CFD_MC_ERR_OUTPUT_SOURCE    = 0x0012,
    E_CFD_MC_ERR_OUTPUT_FORMAT    = 0x0013,

    E_CFD_MC_ERR_OUTPUT_DATAFORMAT  = 0x0014,
    E_CFD_MC_ERR_OUTPUT_ISFULLRANGE = 0x0015,

    E_CFD_MC_ERR_OUTPUT_HDRMODE = 0x0016,

    E_CFD_MC_ERR_HDMIOutput_GammutMapping_Mode = 0x0017,
    E_CFD_MC_ERR_HDMIOutput_GammutMapping_MethodMode = 0x0018,
    E_CFD_MC_ERR_MMInput_ColorimetryHandle_Mode = 0x0019,
    E_CFD_MC_ERR_PanelOutput_GammutMapping_Mode = 0x001a,
    E_CFD_MC_ERR_TMO_TargetRefer_Mode = 0x001b,
    E_CFD_MC_ERR_Target_Max_Luminance = 0x001c,
    E_CFD_MC_ERR_Target_Med_Luminance = 0x001d,
    E_CFD_MC_ERR_Target_Min_Luminance = 0x001e,
    E_CFD_MC_ERR_Source_Max_Luminance = 0x001f,
    E_CFD_MC_ERR_Source_Med_Luminance = 0x0020,
    E_CFD_MC_ERR_Source_Min_Luminance = 0x0021,

    E_CFD_MC_ERR_INPUT_HDRIPMODE_HDRMODE_RULE_VIOLATION = 0X0040,
    E_CFD_MC_ERR_INPUT_FORMAT_DATAFORMAT_NOT_MATCH,
    E_CFD_MC_ERR_INPUT_MAIN_CONTROLS,

    //MM starts from 0x0100
    E_CFD_MC_ERR_MM_Colour_primaries = 0x0100,
    E_CFD_MC_ERR_MM_Transfer_Characteristics = 0x0101,
    E_CFD_MC_ERR_MM_Matrix_Coeffs = 0x0102,
    E_CFD_MC_ERR_MM_IsFullRange = 0x0103,
    E_CFD_MC_ERR_MM_Mastering_Display = 0x0104,
    E_CFD_MC_ERR_MM_Mastering_Display_Coordinates = 0x0105,
    E_CFD_MC_ERR_MM_Handle_Undefined_Case = 0x0106,
    E_CFD_MC_ERR_MM_SEI_Defined = 0x0107,/// HDR10plus
    E_CFD_MC_ERR_MM_SEI_Setting = 0x0108,/// HDR10plus
    E_CFD_MC_ERR_MM_SEI_OverRange = 0x0109,/// HDR10plus

    //HDMI EDID starts from 0x0200
    E_CFD_MC_ERR_HDMI_EDID = 0x0200,
    E_CFD_MC_ERR_HDMI_EDID_Mastering_Display_Coordinates = 0x0201,

    //HDMI InfoFrame starts from 0x0280,
    E_CFD_MC_ERR_HDMI_INFOFRAME = 0x0280,
    E_CFD_MC_ERR_HDMI_INFOFRAME_HDR_Infor = 0x0281,
    E_CFD_MC_ERR_HDMI_INFOFRAME_Mastering_Display_Coordinates = 0x0282,
    E_CFD_MC_ERR_HDMI_INFOFRAME_VSIF_Defined = 0x0283,/// HDR10plus
    E_CFD_MC_ERR_HDMI_INFOFRAME_VSIF_Setting = 0x0284,/// HDR10plus
    E_CFD_MC_ERR_HDMI_INFOFRAME_VSIF_OverRange = 0x0285,/// HDR10plus

    //Panel starts from 0x0300
    E_CFD_MC_ERR_Panel_infor = 0x0300,
    E_CFD_MC_ERR_Panel_infor_Mastering_Display_Coordinates = 0x0301,

    //OSD starts from 0x380
    E_CFD_MC_ERR_UI_infor = 0x0380,

    //HW control starts from 0x390
    E_CFD_MC_ERR_HW_Main_Param = 0x0390,

    //controls for HW
    E_CFD_MC_ERR_HW_IPS_PARAM_OVERRANGE = 0x03A0, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_HDR_OVERRANGE = 0x03A1, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_SDR_OVERRANGE = 0x03A2, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_TMO_OVERRANGE = 0x03A3, //parameters for HW IP control
    E_CFD_MC_ERR_HW_IPS_PARAM_DLC_OVERRANGE = 0x03A4, //parameters for HW IP control

    //interrelation of API
    E_CFD_MC_ERR_INPUT_MAIN_CONTROLS_MM_INTERRELATION,
    E_CFD_MC_ERR_INPUT_MAIN_CONTROLS_HDMI_INTERRELATION,
    E_CFD_MC_ERR_OUTPUT_PANEL_SDRIPMODE_INTERRELATION,

    //HW IP Capability
    E_CFD_MC_ERR_HW_IPS_BT2020CLtoNCL_NOTSUPPORTED = 0x0400,
    E_CFD_MC_ERR_HW_IPS_GMforXVYCC_NOTSUPPORTED = 0x0401,
    E_CFD_MC_ERR_HW_IPS_SDR2HDR_NOTSUPPORTED = 0x0402, //no SDR to HDR now
    E_CFD_MC_ERR_HW_IPS_HDRXtoHDRY_NOTSUPPORTED = 0x0403,
    E_CFD_MC_ERR_HW_IPS_GM_NOTSUPPORTED = 0x0404,
    E_CFD_MC_ERR_HW_IPS_TMO_NOTSUPPORTED = 0x0404,

    //
    //E_CFD_MC_ERR_BYPASS       , //STB/TV can not handle this case, so force all IP bypass
    //E_CFD_MC_ERR_CONTROLINPUT,//something wrong in setting for STU_CFDAPI_MAIN_CONTROL, please check settings
    //E_CFD_MC_ERR_CONTROL_MM_INPUT , ////something wrong in parameters of structures of main control and MM
    //E_CFD_MC_ERR_MMINPUT_FORCE709   , //force input_format to BT709, some information is not avaliable for MM
    //E_CFD_MC_ERR_MMINPUT0   ,
    //E_CFD_MC_ERR_HDMIINPUT  , //this function can not support current HDMI input
    //E_CFD_MC_ERR_ANALOGINPUT , //this function can not support current Analog input
    //E_CFD_MC_ERR_WRONGINPUT  , //this function can not support current input
    //E_CFD_MC_ERR_WRONGINPUTSOURCE  ,//can not support current input source, check the value of u8Input_Source
    E_CFD_MC_ERR_WRONGOUTPUTSOURCE  ,//can not support current output source, check the value of u8Output_Source
    E_CFD_MC_ERR_PROCESSOFF   , //process off
    //E_CFD_MC_ERR_NOSDR2HDRNOW , //no SDR to HDR now, check u8Input_HDRMode & u8Output_HDRMode
    //E_CFD_MC_ERR_WRONGTMOSET  , //not support such TMO , check u8Input_HDRMode & u8Output_HDRMode
    //E_CFD_MC_ERR_PARTIALWRONG , //STB/TV can not handle this case, so force all IP bypass
    //E_CFD_MC_ERR_HW_NOT_SUPPORT_THIS_INPUT, //for current HW, this input format can not supported
    //E_CFD_MC_ERR_HW_NOT_SUPPORT_THIS_OUTPUT, //for current HW, this input format can not supported

    //HW IP process function from Ali
    E_CFD_MC_ERR_HW_IP_PARAMETERS, //check the parameters from CFD to IP process functions from Ali

    //E_CFD_MC_ERR_DLCIP_PARAMETERS, //need to check the parameters for DLC IPs, setting is out of range
    //E_CFD_MC_ERR_0x1001_ERR_IN_Input_Analog_SetConfigures = 0x1001, //some errors happens in Input_Analog_SetConfigures(), please debug

    E_CFD_ERR_PARAM_OVERRANGE,
    E_CFD_ERR_NULLPOINTER,

    E_CFD_MC_ERR_EMUEND

} E_CFD_API_Status; //E_CFD_MC_ERR;

typedef enum _E_CFD_MC_SOURCE
{
    //include VDEC series
    //E_CFD_MC_SOURCE_MM        = 0x00,
    //E_CFD_MC_SOURCE_HDMI  = 0x01,
    //E_CFD_MC_SOURCE_ANALOG    = 0x02,
    //E_CFD_MC_SOURCE_PANEL = 0x03,
    //E_CFD_MC_SOURCE_DVI     = 0x04,
    //E_CFD_MC_SOURCE_ULSA  = 0x04,
    //E_CFD_MC_SOURCE_RESERVED_START,

    /// VGA
    E_CFD_INPUT_SOURCE_VGA     =0x00,

    /// ATV
    E_CFD_INPUT_SOURCE_TV      =0x01,

    /// CVBS
    E_CFD_INPUT_SOURCE_CVBS    =0x02,

    /// S-video
    E_CFD_INPUT_SOURCE_SVIDEO  =0x03,

    /// Component
    E_CFD_INPUT_SOURCE_YPBPR   =0x04,

    /// Scart
    E_CFD_INPUT_SOURCE_SCART   =0x05,

    /// HDMI
    E_CFD_INPUT_SOURCE_HDMI    =0x06,

    /// DTV
    E_CFD_INPUT_SOURCE_DTV     =0x07,

    /// DVI
    E_CFD_INPUT_SOURCE_DVI     =0x08,

    // Application source
    /// Storage
    E_CFD_INPUT_SOURCE_STORAGE =0x09,

    /// KTV
    E_CFD_INPUT_SOURCE_KTV     =0x0A,

    /// JPEG
    E_CFD_INPUT_SOURCE_JPEG    =0x0B,

    //RX for ulsa
    E_CFD_INPUT_SOURCE_RX      =0x0C,

    //For general usage
    E_CFD_INPUT_SOURCE_GENERAL =0x0D,

    /// The max support number of PQ input source
    E_CFD_INPUT_SOURCE_RESERVED_START,

    /// None
    E_CFD_INPUT_SOURCE_NONE = E_CFD_INPUT_SOURCE_RESERVED_START,


} E_CFD_INPUT_SOURCE;

//for output source
typedef enum _E_CFD_OUTPUT_SOURCE
{
    //include VDEC series
    E_CFD_OUTPUT_SOURCE_MM      = 0x00,
    E_CFD_OUTPUT_SOURCE_HDMI    = 0x01,
    E_CFD_OUTPUT_SOURCE_ANALOG  = 0x02,
    E_CFD_OUTPUT_SOURCE_PANEL   = 0x03,
    E_CFD_OUTPUT_SOURCE_ULSA    = 0x04,
    E_CFD_OUTPUT_SOURCE_GENERAL = 0x05,
    E_CFD_OUTPUT_SOURCE_RESERVED_START,

} E_CFD_OUTPUT_SOURCE;
/*
//follow Tomato's table
typedef enum _E_CFD_CFIO
{
    E_CFD_CFIO_RGB_NOTSPECIFIED      = 0x0, //means RGB, but no specific colorspace
    E_CFD_CFIO_RGB_BT601_625         = 0x1,
    E_CFD_CFIO_RGB_BT601_525         = 0x2,
    E_CFD_CFIO_RGB_BT709             = 0x3,
    E_CFD_CFIO_RGB_BT2020            = 0x4,
    E_CFD_CFIO_SRGB                  = 0x5,
    E_CFD_CFIO_ADOBE_RGB             = 0x6,
    E_CFD_CFIO_YUV_NOTSPECIFIED      = 0x7, //means RGB, but no specific colorspace
    E_CFD_CFIO_YUV_BT601_625         = 0x8,
    E_CFD_CFIO_YUV_BT601_525         = 0x9,
    E_CFD_CFIO_YUV_BT709             = 0xA,
    E_CFD_CFIO_YUV_BT2020_NCL        = 0xB,
    E_CFD_CFIO_YUV_BT2020_CL         = 0xC,
    E_CFD_CFIO_XVYCC_601             = 0xD,
    E_CFD_CFIO_XVYCC_709             = 0xE,
    E_CFD_CFIO_SYCC601               = 0xF,
    E_CFD_CFIO_ADOBE_YCC601          = 0x10,
    E_CFD_CFIO_DOLBY_HDR_TEMP        = 0x11,
    E_CFD_CFIO_SYCC709               = 0x12,
    E_CFD_CFIO_DCIP3_THEATER         = 0x13, /// HDR10+
    E_CFD_CFIO_DCIP3_D65             = 0x14, /// HDR10+
    E_CFD_CFIO_RESERVED_START

} E_CFD_CFIO;

typedef enum _E_CFD_MC_FORMAT
{

    E_CFD_MC_FORMAT_RGB     = 0x00,
    E_CFD_MC_FORMAT_YUV422  = 0x01,
    E_CFD_MC_FORMAT_YUV444  = 0x02,
    E_CFD_MC_FORMAT_YUV420  = 0x03,
    E_CFD_MC_FORMAT_RESERVED_START

} E_CFD_MC_FORMAT;

typedef enum _E_CFD_CFIO_RANGE//u8MM_Codec
{
    E_CFD_CFIO_RANGE_LIMIT   = 0x0,
    E_CFD_CFIO_RANGE_FULL    = 0x1,
    E_CFD_CFIO_RANGE_RESERVED_START

} E_CFD_CFIO_RANGE;
*/
typedef enum _E_CFD_HDR_STATUS
{
    E_CFIO_MODE_SDR           = 0x0,
    E_CFIO_MODE_HDR1          = 0x1, // Dolby
    E_CFIO_MODE_HDR2          = 0x2, // open HDR
    E_CFIO_MODE_HDR3          = 0x3, // Hybrid log gamma
    E_CFIO_MODE_HDR4          = 0x4, // TCH
    E_CFIO_MODE_HDR5          = 0x5, // HDR10plus
    E_CFIO_MODE_CONTROL_FLAG_RESET= 0x10,// flag reset
    E_CFIO_MODE_RESERVED_START

} E_CFD_HDR_STATUS;

//ColorPrimary
//use order in HEVC spec and add AdobeRGB
typedef enum _E_CFD_CFIO_CP
{
    E_CFD_CFIO_CP_RESERVED0             = 0x0, //means RGB, but no specific colorspace
    E_CFD_CFIO_CP_BT709_SRGB_SYCC       = 0x1,
    E_CFD_CFIO_CP_UNSPECIFIED           = 0x2,
    E_CFD_CFIO_CP_RESERVED3             = 0x3,
    E_CFD_CFIO_CP_BT470_6               = 0x4,
    E_CFD_CFIO_CP_BT601625              = 0x5,
    E_CFD_CFIO_CP_BT601525_SMPTE170M    = 0x6,
    E_CFD_CFIO_CP_SMPTE240M             = 0x7,
    E_CFD_CFIO_CP_GENERIC_FILM          = 0x8,
    E_CFD_CFIO_CP_BT2020                = 0x9,
    E_CFD_CFIO_CP_CIEXYZ                = 0xA,
    E_CFD_CFIO_CP_DCIP3_THEATER         = 0xB,
    E_CFD_CFIO_CP_DCIP3_D65             = 0xC,
    // 13-21: reserved
    E_CFD_CFIO_CP_EBU3213               = 0x16,
    // 23-127: reserved
    E_CFD_CFIO_CP_ADOBERGB              = 0x80,
    E_CFD_CFIO_CP_PANEL                 = 0x81,
    E_CFD_CFIO_CP_SOURCE                = 0x82,
    E_CFD_CFIO_CP_RESERVED_START

} E_CFD_CFIO_CP;

//Transfer characteristics
//use order in HEVC spec and add AdobeRGB
typedef enum _E_CFD_CFIO_TR
{
    E_CFD_CFIO_TR_RESERVED0             = 0x0, //means RGB, but no specific colorspace
    E_CFD_CFIO_TR_BT709                 = 0x1,
    E_CFD_CFIO_TR_UNSPECIFIED           = 0x2,
    E_CFD_CFIO_TR_RESERVED3             = 0x3,
    E_CFD_CFIO_TR_GAMMA2P2              = 0x4,
    E_CFD_CFIO_TR_GAMMA2P8              = 0x5,
    E_CFD_CFIO_TR_BT601525_601625       = 0x6,
    E_CFD_CFIO_TR_SMPTE240M             = 0x7,
    E_CFD_CFIO_TR_LINEAR                = 0x8,
    E_CFD_CFIO_TR_LOG0                  = 0x9,
    E_CFD_CFIO_TR_LOG1                  = 0xA,
    E_CFD_CFIO_TR_XVYCC                 = 0xB,
    E_CFD_CFIO_TR_BT1361                = 0xC,
    E_CFD_CFIO_TR_SRGB_SYCC             = 0xD,
    E_CFD_CFIO_TR_BT2020NCL             = 0xE,
    E_CFD_CFIO_TR_BT2020CL              = 0xF,
    E_CFD_CFIO_TR_SMPTE2084             = 0x10,
    E_CFD_CFIO_TR_SMPTE428              = 0x11,
    E_CFD_CFIO_TR_HLG                   = 0x12,
    E_CFD_CFIO_TR_BT1886                = 0x13,
    E_CFD_CFIO_TR_DOLBYMETA             = 0x14,
    E_CFD_CFIO_TR_ADOBERGB              = 0x15,
    E_CFD_CFIO_TR_GAMMA2P6              = 0x16,
    E_CFD_CFIO_TR_RESERVED_START

} E_CFD_CFIO_TR;

//Matrix coefficient
//use order in HEVC spec
typedef enum _E_CFD_CFIO_MC
{
    E_CFD_CFIO_MC_IDENTITY                  = 0x0, //means RGB, but no specific colorspace
    E_CFD_CFIO_MC_BT709_XVYCC709            = 0x1,
    E_CFD_CFIO_MC_UNSPECIFIED               = 0x2,
    E_CFD_CFIO_MC_RESERVED                  = 0x3,
    E_CFD_CFIO_MC_USFCCT47                  = 0x4,
    E_CFD_CFIO_MC_BT601625_XVYCC601_SYCC    = 0x5,
    E_CFD_CFIO_MC_BT601525_SMPTE170M        = 0x6,
    E_CFD_CFIO_MC_SMPTE240M                 = 0x7,
    E_CFD_CFIO_MC_YCGCO                     = 0x8,
    E_CFD_CFIO_MC_BT2020NCL                 = 0x9,
    E_CFD_CFIO_MC_BT2020CL                  = 0xA,
    E_CFD_CFIO_MC_YDZDX                     = 0xB,
    E_CFD_CFIO_MC_CD_NCL                    = 0xC,
    E_CFD_CFIO_MC_CD_CL                     = 0xD,
    E_CFD_CFIO_MC_ICTCP                     = 0xE,
    E_CFD_CFIO_MC_RESERVED_START

} E_CFD_CFIO_MC;

typedef struct{
    MS_U8 u8BlackLevel;
    MS_U8 u8WhiteLevel;
}ST_KDRV_XC_CFD_BLACKANDWHITE;

#endif  //_MDRV_GFLIP_CFD_H