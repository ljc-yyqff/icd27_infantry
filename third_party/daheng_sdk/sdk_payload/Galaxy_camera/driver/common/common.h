#ifndef GX_COMMON_H
#define GX_COMMON_H

#define DEV_TYPE_NAME_CONTROL    "control"
#define DEV_TYPE_NAME_EVENT      "event"
#define DEV_TYPE_NAME_STREAM     "stream"
#define DEV_TYPE_NAME_CARD       "card"
#define SEP_CHAN_NUM             "_"

#define CARD_MAX_NUM_BARS 3
#define LINE_INFO_NUM_MAX 8

typedef int NTSTATUS;
#define NT_SUCCESS(Status)               (((NTSTATUS)(Status)) >= 0)
#define STATUS_SUCCESS                   ((NTSTATUS)0x00000000L)
#define STATUS_UNSUCCESSFUL              ((NTSTATUS)0xC0000001L)
#define STATUS_NOT_IMPLEMENTED           ((NTSTATUS)0xC0000002L)
#define STATUS_FAIL_CHECK                ((NTSTATUS)0xC0000229L)
#define STATUS_INVALID_PARAMETER         ((NTSTATUS)0xC000000DL)    // winnt
#define STATUS_INTERNAL_ERROR            ((NTSTATUS)0xC00000E5L)
#define STATUS_NOT_SUPPORTED             ((NTSTATUS)0xC00000BBL)
#define STATUS_TIMEOUT                   ((NTSTATUS)0x00000102L)    // winnt
#define STATUS_NOINTERFACE               ((NTSTATUS)0xC00002B9L)
#define STATUS_HV_NO_RESOURCES           ((NTSTATUS)0xC035001DL)
#define STATUS_IO_DEVICE_ERROR           ((NTSTATUS)0xC0000185L)
#define STATUS_NO_WORK_DONE              ((NTSTATUS)0x80000032L)
#define STATUS_NO_MEMORY                 ((NTSTATUS)0xC0000017L)    // winnt
#define STATUS_NO_MORE_ENTRIES           ((NTSTATUS)0x8000001AL)
#define STATUS_DRIVER_INTERNAL_ERROR     ((NTSTATUS)0xC0000183L)

enum GX_POLL_TYPE
{
    GX_FOLL_WRITE = 0,
    GX_FOLL_FORCE,
    GX_FOLL_LONGTERM
};

enum TRACE_LEVEL
{
    TRACE_LEVEL_DEBUG = 0,
    TRACE_LEVEL_INFO,
    TRACE_LEVEL_WARN,
    TRACE_LEVEL_ERROR,
};

void _TraceFormat(int level, const char* fmt, ...);

#define TraceVerbose(x, fmt, args...)       _TraceFormat(TRACE_LEVEL_DEBUG, fmt, ##args)
#define TraceDebug(x, fmt, args...)         _TraceFormat(TRACE_LEVEL_DEBUG, fmt, ##args)
#define TraceInfo(x, fmt, args...)          _TraceFormat(TRACE_LEVEL_INFO, fmt, ##args)
#define TraceWarning(x, fmt, args...)       _TraceFormat(TRACE_LEVEL_WARN, fmt, ##args)
#define TraceError(x, fmt, args...)         _TraceFormat(TRACE_LEVEL_ERROR, fmt, ##args)

#endif