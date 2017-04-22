#pragma once


#include <mfobjects.h>


#include "videoInput.h"

struct IMFMediaSession;

struct MediaEventCode
{
	enum MediaEvent
	{
		NULLPTR = 0,
		CLOSED = 1,
		OK = 2,
		STATUS_ERROR = 3,
		EXECUTION_ERROR = 4,
		STOPPED = 5
	};
};

class VideoCaptureSession: public IMFAsyncCallback
{
public:
	VideoCaptureSession(void);
	virtual ~VideoCaptureSession(void);

	ResultCode::Result init(IMFMediaSession *pSession, IStopCallback *pIStopCallback);

	HANDLE getStartCompleteEvent();

	HANDLE getCloseCompleteEvent();

	STDMETHODIMP GetParameters(DWORD *pdwFlags, DWORD *pdwQueue);  

    STDMETHODIMP Invoke(IMFAsyncResult* pAsyncResult);

    STDMETHODIMP QueryInterface(REFIID iid, void** ppv);

	STDMETHODIMP_(ULONG) AddRef();
        
	STDMETHODIMP_(ULONG) Release();

private:

	MediaEventCode::MediaEvent processMediaEvent(CComPtr<IMFMediaEvent>& pMediaEvent);


	volatile long refCount;  

	CComAutoCriticalSection critSec;

	CComPtr<IMFMediaSession> pLocalSession;

	HANDLE closeCompleteEvent;

	HANDLE startCompleteEvent;

	IStopCallback *pLocalIStopCallback;
};

