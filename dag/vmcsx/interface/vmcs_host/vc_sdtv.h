/*=============================================================================
Copyright (c) 2010 Broadcom Europe Limited.
All rights reserved.

Project  :  TV service
Module   :  Common host interface for SDTV
File     :  $Id: //software/customers/broadcom/mobcom/vc4/rel/interface/vmcs_host/vc_sdtv.h#2 $
Revision :  $Revision: #2 $

FILE DESCRIPTION
SDTV common host header for TV service
=============================================================================*/
#ifndef _VC_SDTV_H_
#define _VC_SDTV_H_

/** Different SDTV modes */
/** colour */
typedef enum SDTV_COLOUR_
{
   SDTV_COLOUR_UNKNOWN = 0x0,
   SDTV_COLOUR_RGB     = 0x4,
   SDTV_COLOUR_YPRPB   = 0x8
} SDTV_COLOUR_T;
/** operation mode */
typedef enum SDTV_MODE_T_
{
   SDTV_MODE_NTSC       = 0, /**<Normal NTSC */
   SDTV_MODE_NTSC_J     = 1, /**<Japanese version of NTSC - no pedestal.*/
   SDTV_MODE_PAL        = 2, /**<Normal PAL */
   SDTV_MODE_PAL_M      = 3, /**<Brazilian version of PAL - 525/60 rather than 625/50, different subcarrier */
   SDTV_MODE_FORMAT_MASK = 0x3,
   
   SDTV_MODE_OUTPUT_MASK = 0xc,

   SDTV_MODE_YPRPB_480i = (SDTV_MODE_NTSC | SDTV_COLOUR_YPRPB),
   SDTV_MODE_RGB_480i   = (SDTV_MODE_NTSC | SDTV_COLOUR_RGB),
   SDTV_MODE_YPRPB_576i = (SDTV_MODE_PAL  | SDTV_COLOUR_YPRPB),
   SDTV_MODE_RGB_576i   = (SDTV_MODE_PAL  | SDTV_COLOUR_RGB),

   SDTV_MODE_OFF        = 0xf0
} SDTV_MODE_T;

/** Different aspect ratios */
typedef enum SDTV_ASPECT_T_
{
   // TODO: extend this to allow picture placement/size to be communicated.
   SDTV_ASPECT_UNKNOWN  = 0, /**<Unknown */
   SDTV_ASPECT_4_3      = 1, /**<4:3 */
   SDTV_ASPECT_14_9     = 2, /**<14:9 */
   SDTV_ASPECT_16_9     = 3  /**<16:9 */
} SDTV_ASPECT_T;

/** SDTV power on option */
typedef struct SDTV_OPTIONS_T_
{
   SDTV_ASPECT_T   aspect;
} SDTV_OPTIONS_T;

/**
 * Different copy protection modes
 * At the moment we have only implemented Macrovision
 */
typedef enum
{
   SDTV_CP_NONE              = 0, /**<No copy protection */
   SDTV_CP_MACROVISION_TYPE1 = 1, /**<Macrovision Type 1 */
   SDTV_CP_MACROVISION_TYPE2 = 2, /**<Macrovision Type 2 */
   SDTV_CP_MACROVISION_TYPE3 = 3, /**<Macrovision Type 3 */
   SDTV_CP_MACROVISION_TEST1 = 4, /**<Macrovision Test 1 */
   SDTV_CP_MACROVISION_TEST2 = 5, /**<Macrovision Test 2 */
   SDTV_CP_CGMS_COPYFREE     = 6, /**<CGMS copy freely */
   SDTV_CP_CGMS_COPYNOMORE   = 7, /**<CGMS copy no more */
   SDTV_CP_CGMS_COPYONCE     = 8, /**<CGMS copy once */
   SDTV_CP_CGMS_COPYNEVER    = 9  /**<CGMS copy never */
} SDTV_CP_MODE_T;

/**
 * SDTV notifications
 */
typedef enum 
{
   VC_SDTV_UNPLUGGED          = 1 << 16, /**<SDTV cable unplugged, subject to platform support */
   VC_SDTV_STANDBY            = 1 << 17, /**<SDTV cable is plugged in */
   VC_SDTV_NTSC               = 1 << 18, /**<SDTV is in NTSC mode */
   VC_SDTV_PAL                = 1 << 19, /**<SDTV is in PAL mode */
   VC_SDTV_CP_INACTIVE        = 1 << 20, /**<Copy protection disabled */
   VC_SDTV_CP_ACTIVE          = 1 << 21  /**<Copy protection enabled */
} VC_SDTV_NOTIFY_T;

/**
 * Callback reason and arguments from vec middleware
 * Each callback comes with two optional uint32_t parameters.
 * Reason                     param1       param2      remark
 * VC_SDTV_UNPLUGGED            -            -         cable is unplugged
 * VC_SDTV_STANDBY              -            -         cable is plugged in
 * VC_SDTV_NTSC              SDTV_MODE_T SDTV_ASPECT_T NTSC mode active with said aspect ratio
 * VC_SDTV_PAL               SDTV_MODE_T SDTV_ASPECT_T PAL  mode active with said aspect ratio
 * VC_SDTV_CP_INACTIVE          -            -         copy protection is inactive
 * VC_SDTV_CP_ACTIVE         SDTV_CP_MODE_T  -         copy protection is active
 */

#endif