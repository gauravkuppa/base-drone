/* AUTOGENERATED FILE. DO NOT EDIT. */
#ifndef _MOCKPERIODIC_CALLBACKS_H
#define _MOCKPERIODIC_CALLBACKS_H

#include "unity.h"
#include "periodic_callbacks.h"
#include "CException.h"

/* Ignore the following warnings, since we are copying code */
#if defined(__GNUC__) && !defined(__ICC) && !defined(__TMS470__)
#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 6 || (__GNUC_MINOR__ == 6 && __GNUC_PATCHLEVEL__ > 0)))
#pragma GCC diagnostic push
#endif
#if !defined(__clang__)
#pragma GCC diagnostic ignored "-Wpragmas"
#endif
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wduplicate-decl-specifier"
#endif

void Mockperiodic_callbacks_Init(void);
void Mockperiodic_callbacks_Destroy(void);
void Mockperiodic_callbacks_Verify(void);




#define periodic_callbacks__initialize_Ignore() periodic_callbacks__initialize_CMockIgnore()
void periodic_callbacks__initialize_CMockIgnore(void);
#define periodic_callbacks__initialize_Expect() periodic_callbacks__initialize_CMockExpect(__LINE__)
void periodic_callbacks__initialize_CMockExpect(UNITY_LINE_TYPE cmock_line);
typedef void (* CMOCK_periodic_callbacks__initialize_CALLBACK)(int cmock_num_calls);
void periodic_callbacks__initialize_AddCallback(CMOCK_periodic_callbacks__initialize_CALLBACK Callback);
void periodic_callbacks__initialize_Stub(CMOCK_periodic_callbacks__initialize_CALLBACK Callback);
#define periodic_callbacks__initialize_StubWithCallback periodic_callbacks__initialize_Stub
#define periodic_callbacks__initialize_ExpectAndThrow(cmock_to_throw) periodic_callbacks__initialize_CMockExpectAndThrow(__LINE__, cmock_to_throw)
void periodic_callbacks__initialize_CMockExpectAndThrow(UNITY_LINE_TYPE cmock_line, CEXCEPTION_T cmock_to_throw);
#define periodic_callbacks__1Hz_Ignore() periodic_callbacks__1Hz_CMockIgnore()
void periodic_callbacks__1Hz_CMockIgnore(void);
#define periodic_callbacks__1Hz_ExpectAnyArgs() periodic_callbacks__1Hz_CMockExpectAnyArgs(__LINE__)
void periodic_callbacks__1Hz_CMockExpectAnyArgs(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__1Hz_Expect(callback_count) periodic_callbacks__1Hz_CMockExpect(__LINE__, callback_count)
void periodic_callbacks__1Hz_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t callback_count);
typedef void (* CMOCK_periodic_callbacks__1Hz_CALLBACK)(uint32_t callback_count, int cmock_num_calls);
void periodic_callbacks__1Hz_AddCallback(CMOCK_periodic_callbacks__1Hz_CALLBACK Callback);
void periodic_callbacks__1Hz_Stub(CMOCK_periodic_callbacks__1Hz_CALLBACK Callback);
#define periodic_callbacks__1Hz_StubWithCallback periodic_callbacks__1Hz_Stub
#define periodic_callbacks__1Hz_ExpectAndThrow(callback_count, cmock_to_throw) periodic_callbacks__1Hz_CMockExpectAndThrow(__LINE__, callback_count, cmock_to_throw)
void periodic_callbacks__1Hz_CMockExpectAndThrow(UNITY_LINE_TYPE cmock_line, uint32_t callback_count, CEXCEPTION_T cmock_to_throw);
#define periodic_callbacks__1Hz_IgnoreArg_callback_count() periodic_callbacks__1Hz_CMockIgnoreArg_callback_count(__LINE__)
void periodic_callbacks__1Hz_CMockIgnoreArg_callback_count(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__10Hz_Ignore() periodic_callbacks__10Hz_CMockIgnore()
void periodic_callbacks__10Hz_CMockIgnore(void);
#define periodic_callbacks__10Hz_ExpectAnyArgs() periodic_callbacks__10Hz_CMockExpectAnyArgs(__LINE__)
void periodic_callbacks__10Hz_CMockExpectAnyArgs(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__10Hz_Expect(callback_count) periodic_callbacks__10Hz_CMockExpect(__LINE__, callback_count)
void periodic_callbacks__10Hz_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t callback_count);
typedef void (* CMOCK_periodic_callbacks__10Hz_CALLBACK)(uint32_t callback_count, int cmock_num_calls);
void periodic_callbacks__10Hz_AddCallback(CMOCK_periodic_callbacks__10Hz_CALLBACK Callback);
void periodic_callbacks__10Hz_Stub(CMOCK_periodic_callbacks__10Hz_CALLBACK Callback);
#define periodic_callbacks__10Hz_StubWithCallback periodic_callbacks__10Hz_Stub
#define periodic_callbacks__10Hz_ExpectAndThrow(callback_count, cmock_to_throw) periodic_callbacks__10Hz_CMockExpectAndThrow(__LINE__, callback_count, cmock_to_throw)
void periodic_callbacks__10Hz_CMockExpectAndThrow(UNITY_LINE_TYPE cmock_line, uint32_t callback_count, CEXCEPTION_T cmock_to_throw);
#define periodic_callbacks__10Hz_IgnoreArg_callback_count() periodic_callbacks__10Hz_CMockIgnoreArg_callback_count(__LINE__)
void periodic_callbacks__10Hz_CMockIgnoreArg_callback_count(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__100Hz_Ignore() periodic_callbacks__100Hz_CMockIgnore()
void periodic_callbacks__100Hz_CMockIgnore(void);
#define periodic_callbacks__100Hz_ExpectAnyArgs() periodic_callbacks__100Hz_CMockExpectAnyArgs(__LINE__)
void periodic_callbacks__100Hz_CMockExpectAnyArgs(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__100Hz_Expect(callback_count) periodic_callbacks__100Hz_CMockExpect(__LINE__, callback_count)
void periodic_callbacks__100Hz_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t callback_count);
typedef void (* CMOCK_periodic_callbacks__100Hz_CALLBACK)(uint32_t callback_count, int cmock_num_calls);
void periodic_callbacks__100Hz_AddCallback(CMOCK_periodic_callbacks__100Hz_CALLBACK Callback);
void periodic_callbacks__100Hz_Stub(CMOCK_periodic_callbacks__100Hz_CALLBACK Callback);
#define periodic_callbacks__100Hz_StubWithCallback periodic_callbacks__100Hz_Stub
#define periodic_callbacks__100Hz_ExpectAndThrow(callback_count, cmock_to_throw) periodic_callbacks__100Hz_CMockExpectAndThrow(__LINE__, callback_count, cmock_to_throw)
void periodic_callbacks__100Hz_CMockExpectAndThrow(UNITY_LINE_TYPE cmock_line, uint32_t callback_count, CEXCEPTION_T cmock_to_throw);
#define periodic_callbacks__100Hz_IgnoreArg_callback_count() periodic_callbacks__100Hz_CMockIgnoreArg_callback_count(__LINE__)
void periodic_callbacks__100Hz_CMockIgnoreArg_callback_count(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__1000Hz_Ignore() periodic_callbacks__1000Hz_CMockIgnore()
void periodic_callbacks__1000Hz_CMockIgnore(void);
#define periodic_callbacks__1000Hz_ExpectAnyArgs() periodic_callbacks__1000Hz_CMockExpectAnyArgs(__LINE__)
void periodic_callbacks__1000Hz_CMockExpectAnyArgs(UNITY_LINE_TYPE cmock_line);
#define periodic_callbacks__1000Hz_Expect(callback_count) periodic_callbacks__1000Hz_CMockExpect(__LINE__, callback_count)
void periodic_callbacks__1000Hz_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t callback_count);
typedef void (* CMOCK_periodic_callbacks__1000Hz_CALLBACK)(uint32_t callback_count, int cmock_num_calls);
void periodic_callbacks__1000Hz_AddCallback(CMOCK_periodic_callbacks__1000Hz_CALLBACK Callback);
void periodic_callbacks__1000Hz_Stub(CMOCK_periodic_callbacks__1000Hz_CALLBACK Callback);
#define periodic_callbacks__1000Hz_StubWithCallback periodic_callbacks__1000Hz_Stub
#define periodic_callbacks__1000Hz_ExpectAndThrow(callback_count, cmock_to_throw) periodic_callbacks__1000Hz_CMockExpectAndThrow(__LINE__, callback_count, cmock_to_throw)
void periodic_callbacks__1000Hz_CMockExpectAndThrow(UNITY_LINE_TYPE cmock_line, uint32_t callback_count, CEXCEPTION_T cmock_to_throw);
#define periodic_callbacks__1000Hz_IgnoreArg_callback_count() periodic_callbacks__1000Hz_CMockIgnoreArg_callback_count(__LINE__)
void periodic_callbacks__1000Hz_CMockIgnoreArg_callback_count(UNITY_LINE_TYPE cmock_line);

#if defined(__GNUC__) && !defined(__ICC) && !defined(__TMS470__)
#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 6 || (__GNUC_MINOR__ == 6 && __GNUC_PATCHLEVEL__ > 0)))
#pragma GCC diagnostic pop
#endif
#endif

#endif
