/*******************************************************************************
Copyright � 2016, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _VL53L0X_API_CORE_H_
#define _VL53L0X_API_CORE_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"


#ifdef __cplusplus
extern "C" {
#endif


VL53L0X_Error VL53L0X_reverse_bytes(uint8 *data, uint32 size);

VL53L0X_Error VL53L0X_measurement_poll_for_completion(VL53L0X_DEV Dev);

uint8 VL53L0X_encode_vcsel_period(uint8 vcsel_period_pclks);

uint8 VL53L0X_decode_vcsel_period(uint8 vcsel_period_reg);

uint32 VL53L0X_isqrt(uint32 num);

uint32 VL53L0X_quadrature_sum(uint32 a, uint32 b);

VL53L0X_Error VL53L0X_get_info_from_device(VL53L0X_DEV Dev, uint8 option);

VL53L0X_Error VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV Dev,
	VL53L0X_VcselPeriod VcselPeriodType, uint8 VCSELPulsePeriodPCLK);

VL53L0X_Error VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV Dev,
	VL53L0X_VcselPeriod VcselPeriodType, uint8 *pVCSELPulsePeriodPCLK);

uint32 VL53L0X_decode_timeout(uint16 encoded_timeout);

VL53L0X_Error get_sequence_step_timeout(VL53L0X_DEV Dev,
			VL53L0X_SequenceStepId SequenceStepId,
			uint32 *pTimeOutMicroSecs);

VL53L0X_Error set_sequence_step_timeout(VL53L0X_DEV Dev,
			VL53L0X_SequenceStepId SequenceStepId,
			uint32 TimeOutMicroSecs);

VL53L0X_Error VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev,
	uint32 MeasurementTimingBudgetMicroSeconds);

VL53L0X_Error VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev,
		uint32 *pMeasurementTimingBudgetMicroSeconds);

VL53L0X_Error VL53L0X_load_tuning_settings(VL53L0X_DEV Dev,
		uint8 *pTuningSettingBuffer);

VL53L0X_Error VL53L0X_calc_sigma_estimate(VL53L0X_DEV Dev,
		VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
		uint32 *pSigmaEstimate, uint32 *pDmax_mm);

VL53L0X_Error VL53L0X_get_total_xtalk_rate(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
	uint32 *ptotal_xtalk_rate_mcps);

VL53L0X_Error VL53L0X_get_total_signal_rate(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
	uint32 *ptotal_signal_rate_mcps);

VL53L0X_Error VL53L0X_get_pal_range_status(VL53L0X_DEV Dev,
		 uint8 DeviceRangeStatus,
		 uint32 SignalRate,
		 uint16 EffectiveSpadRtnCount,
		 VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
		 uint8 *pPalRangeStatus);

uint32 VL53L0X_calc_timeout_mclks(VL53L0X_DEV Dev,
	uint32 timeout_period_us, uint8 vcsel_period_pclks);

uint16 VL53L0X_encode_timeout(uint32 timeout_macro_clks);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L0X_API_CORE_H_ */
