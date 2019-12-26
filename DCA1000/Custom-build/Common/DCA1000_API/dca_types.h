/**
 * @file dca_types.h
 *
 * @author JP
 *
 * @version 0.1
 *
 * @brief This file contains typedef definitions for data types
 *
 * @par
 * NOTE:
 *     (C) Copyright 2019 Texas Instruments, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


///*****************************************************************************
/// HISTORY :
/// VERSION        DATE              AUTHOR      CHANGE DESCRIPTION
/// 0.1            10 Oct 2017       JP          Created
///*****************************************************************************

#ifndef DCA_TYPES
#define DCA_TYPES

///****************
/// Typedefs
///****************

typedef char                             SINT8;
/**< Character type                             : 8 bits                    */
typedef unsigned char                    UINT8;
/**< Unsigned Character type                    : 8 bits                    */
typedef signed long                      SLONG;
/**< Signed long integer type                   : 32 bits                   */
typedef unsigned long                    ULONG;
/**< Unsigned long integer type                 : 32 bits                   */
typedef short                            SINT16;
/**< Signed short integer type                  : 16 bits                   */
typedef unsigned short                   UINT16;
/**< Unsigned short integer type                : 16 bits                   */
typedef int                              SINT32;
/**< Signed integer type                        : 32 bits                   */
typedef unsigned int                     UINT32;
/**< Unsigned integer type                      : 32 bits                   */
typedef float                            FLOAT32;
/**< Float type                                 : 32 bits                   */
typedef unsigned long long int           ULONG64;
/**< Unsigned Long Long type                    : 64 bits                   */
typedef double                           DOUBLE;
/**< Double type                                : 64 bits                   */

#endif // DCA_TYPES

