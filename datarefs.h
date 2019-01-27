//#pragma once
#define MSG_ADD_DATAREF 0x01000000           //  Add dataref to DRE message

#include "XPLMDataAccess.h"
XPLMDataRef gDataRef_AP_Prendido = NULL;          //  DataRef a añadir
XPLMDataRef gDataRef_APModoVertical = NULL;       //  DataRef a añadir
XPLMDataRef gDataRef_APModoLateral = NULL;          //  DataRef a añadir

XPLMDataRef gDataRef_VertSpeed = NULL;          //  DataRef a añadir
XPLMDataRef gDataRef_kPIDalt = NULL;          //  DataRef a añadir
XPLMDataRef gDataRef_dPIDalt = NULL; 
XPLMDataRef gDataRef_APVS = NULL;
XPLMDataRef gDataRef_EtapaVOR = NULL;

XPLMDataRef gDataRef_EstadoLOC = NULL; //Estado del localizador: 0->inactivo, 1->armado, 2->activo

//Variables asociadas a DataRefs
static int g_apPrendido = 0;
static int g_apLateralMode = 0;
static int g_apVerticalMode = 0;
static float g_vertSpeed = 0;
static float g_kPIDAlt = 0;
static float g_dPIDAlt = 0;
static float g_apVS = 0; //El DataRef de XPlane de V/S del piloto automático sólo se puede escribir con algunos aviones, con otros siempre se sobreescribe a cero.
static int g_etapaVOR = 0;
static int g_estadoLOC = 0;

float RegAPPrendidoDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
int GetAPPrendidoDataRefCB(void* inRefcon);
void SetAPPrendidoDataRefCB(void* inRefcon, int inValue);

float RegAPVerticalModeDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
int GetAPVerticalModeDataRefCB(void* inRefcon);
void SetAPVerticalModeDataRefCB(void* inRefcon, int inValue);

float RegAPLateralModeDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
int GetAPLateralModeDataRefCB(void* inRefcon);
void SetAPLateralModeDataRefCB(void* inRefcon, int inValue);

float RegVertSpeedDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
float GetVertSpeedDataRefCB(void* inRefcon);
void SetVertSpeedDataRefCB(void* inRefcon, float inValue);

float RegkPIDAltDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
float GetkPIDAltDataRefCB(void* inRefcon);
void SetkPIDAltDataRefCB(void* inRefcon, float inValue);

float RegdPIDAltDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
float GetdPIDAltDataRefCB(void* inRefcon);
void SetdPIDAltDataRefCB(void* inRefcon, float inValue);

float RegAPVSDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon);  //  Declare callback to register dataref
float GetAPVSDataRefCB(void* inRefcon);
void SetAPVSDataRefCB(void* inRefcon, float inValue);

//Etapas interceptación VOR
float RegEtapaInterceptacionVOR(float elapsedMe, float elapsedSim, int counter, void * refcon);
int GetEtapaInterceptacionVOR(void* inRefcon);
void SetEtapaInterceptacionVOR(void* inRefcon, int inValue);

//Estado del localizador (VOR o ILS)
float RegEstadoLOC(float elapsedMe, float elapsedSim, int counter, void * refcon);
int GetEstadoLOC(void* inRefcon);
void SetEstadoLOC(void* inRefcon, int inValue);



void setupDataRefs()
{
	gDataRef_AP_Prendido = XPLMRegisterDataAccessor("CUSTOM/AP/AP_Prendido",
		xplmType_Int,                                  // The types we support
		1,                                             // Writable
		GetAPPrendidoDataRefCB, SetAPPrendidoDataRefCB,      // Integer accessors
		NULL, NULL,                                    // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

	gDataRef_AP_Prendido = XPLMFindDataRef("CUSTOM/AP/AP_Prendido");
	XPLMSetDatai(gDataRef_AP_Prendido, 0);
	XPLMRegisterFlightLoopCallback(RegAPPrendidoDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE

	gDataRef_APModoVertical = XPLMRegisterDataAccessor("CUSTOM/AP/Vertical/Modo",
		xplmType_Int,                                  // The types we support
		1,                                             // Writable
		GetAPVerticalModeDataRefCB, SetAPVerticalModeDataRefCB,      // Integer accessors
		NULL, NULL,                                    // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

													   // Find and intialize our Counter dataref
	gDataRef_APModoVertical = XPLMFindDataRef("CUSTOM/AP/Vertical/Modo");
	XPLMSetDatai(gDataRef_APModoVertical, 0);
	XPLMRegisterFlightLoopCallback(RegAPVerticalModeDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE

	gDataRef_APModoLateral = XPLMRegisterDataAccessor("CUSTOM/AP/Lateral/Modo",
		xplmType_Int,                                  // The types we support
		1,                                             // Writable
		GetAPLateralModeDataRefCB, SetAPLateralModeDataRefCB,      // Integer accessors
		NULL, NULL,                                    // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

													   // Find and intialize our Counter dataref
	gDataRef_APModoLateral = XPLMFindDataRef("CUSTOM/AP/Lateral/Modo");
	XPLMSetDatai(gDataRef_APModoLateral, 0);
	XPLMRegisterFlightLoopCallback(RegAPLateralModeDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE

	gDataRef_VertSpeed = XPLMRegisterDataAccessor("CUSTOM/Flight/VertSpeed",
		xplmType_Float,                                  // The types we support
		1,                                             // Writable
		NULL, NULL,      // Integer accessors
		GetVertSpeedDataRefCB, SetVertSpeedDataRefCB,                                    // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

													   // Find and intialize our Counter dataref
	gDataRef_APModoLateral = XPLMFindDataRef("CUSTOM/Flight/VertSpeed");
	XPLMRegisterFlightLoopCallback(RegVertSpeedDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE

	gDataRef_kPIDalt = XPLMRegisterDataAccessor("CUSTOM/AP/Vertical/Altitude/kPIDAlt",
		xplmType_Float,                                  // The types we support
		1,                                             // Writable
		NULL, NULL,      // Integer accessors
		GetkPIDAltDataRefCB, SetkPIDAltDataRefCB,                                    // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

													   // Find and intialize our Counter dataref
	gDataRef_kPIDalt = XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt");	
	XPLMRegisterFlightLoopCallback(RegkPIDAltDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE
	XPLMSetDataf(gDataRef_kPIDalt, 0.00025); //0.00025

	gDataRef_dPIDalt = XPLMRegisterDataAccessor("CUSTOM/AP/Vertical/Altitude/dPIDAlt",
		xplmType_Float,                                  // The types we support
		1,                                             // Writable
		NULL, NULL,      // Integer accessors
		GetdPIDAltDataRefCB, SetdPIDAltDataRefCB,                                    // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

													   // Find and intialize our Counter dataref
	gDataRef_dPIDalt = XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt");
	XPLMRegisterFlightLoopCallback(RegdPIDAltDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE
	//XPLMSetDataf(gDataRef_dPIDalt, 0.00025); //0.00025

	gDataRef_APVS = XPLMRegisterDataAccessor("CUSTOM/AP/Vertical/VerticalSpeed",
		xplmType_Float,                                  // The types we support
		1,                                             // Writable
		NULL, NULL,      // Integer accessors
		GetAPVSDataRefCB, SetAPVSDataRefCB,            // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

	gDataRef_APVS = XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed");	
	XPLMRegisterFlightLoopCallback(RegAPVSDataRefInDRE, 1, NULL);   // This FLCB will register our custom dataref in DRE
		
	//Etapas interceptación VOR
	gDataRef_EtapaVOR = XPLMRegisterDataAccessor("CUSTOM/AP/Lateral/EtapaVOR",
		xplmType_Int,                                  // The types we support
		1,                                             // Writable
		GetEtapaInterceptacionVOR, SetEtapaInterceptacionVOR,      // Integer accessors
		NULL, NULL,            // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

	gDataRef_EtapaVOR = XPLMFindDataRef("CUSTOM/AP/Lateral/EtapaVOR");
	XPLMRegisterFlightLoopCallback(RegEtapaInterceptacionVOR, 1, NULL);   // This FLCB will register our custom dataref in DRE

	//Estado LOC
	gDataRef_EstadoLOC = XPLMRegisterDataAccessor("CUSTOM/AP/Lateral/EstadoLOC",
		xplmType_Int,                                  // The types we support
		1,                                             // Writable
		GetEstadoLOC, SetEstadoLOC,      // Integer accessors
		NULL, NULL,            // Float accessors
		NULL, NULL,                                    // Doubles accessors
		NULL, NULL,                                    // Int array accessors
		NULL, NULL,                                    // Float array accessors
		NULL, NULL,                                    // Raw data accessors
		NULL, NULL);                                   // Refcons not used

	gDataRef_EstadoLOC = XPLMFindDataRef("CUSTOM/AP/Lateral/EstadoLOC");
	XPLMRegisterFlightLoopCallback(RegEstadoLOC, 1, NULL);   // This FLCB will register our custom dataref in DRE

}


void tearDownDataRefs()
{
	XPLMUnregisterDataAccessor(gDataRef_APModoLateral);
	XPLMUnregisterFlightLoopCallback(RegAPLateralModeDataRefInDRE, NULL);	 //  Don't forget to unload this callback.

	XPLMUnregisterDataAccessor(gDataRef_VertSpeed);
	XPLMUnregisterFlightLoopCallback(RegVertSpeedDataRefInDRE, NULL);

	XPLMUnregisterDataAccessor(gDataRef_kPIDalt);
	XPLMUnregisterFlightLoopCallback(RegkPIDAltDataRefInDRE, NULL);
}

//AP PRENDIDO
int GetAPPrendidoDataRefCB(void* inRefcon)
{
	return g_apPrendido;
}

void SetAPPrendidoDataRefCB(void* inRefcon, int inValue)
{
	g_apPrendido = inValue;
}

float RegAPPrendidoDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/AP_Prendido");
	}

	return 0;  // Flight loop is called only once!
}
// / AP PRENDIDO

//AP VERTICAL MODE
int GetAPVerticalModeDataRefCB(void* inRefcon)
{
	return g_apVerticalMode;
}

void SetAPVerticalModeDataRefCB(void* inRefcon, int inValue)
{
	g_apVerticalMode = inValue;
}

float RegAPVerticalModeDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Vertical/Modo");
	}

	return 0;  // Flight loop is called only once!
}
// / AP VERTICAL MODE

//AP LATERAL MODE
int GetAPLateralModeDataRefCB(void* inRefcon)
{
	return g_apLateralMode;
}

void SetAPLateralModeDataRefCB(void* inRefcon, int inValue)
{
	g_apLateralMode = inValue;
}

float RegAPLateralModeDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Lateral/Modo");
	}

	return 0;  // Flight loop is called only once!
}
// / AP LATERAL MODE

// VERT SPEED
float GetVertSpeedDataRefCB(void* inRefcon)
{
	return g_vertSpeed;
}

void SetVertSpeedDataRefCB(void* inRefcon, float inValue)
{
	g_vertSpeed = inValue;
}

float RegVertSpeedDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/Flight/VertSpeed");
	}

	return 0;  // Flight loop is called only once!
}
// / VERT SPEED

// K PID ALT
float GetkPIDAltDataRefCB(void* inRefcon)
{
	return g_kPIDAlt;
}

void SetkPIDAltDataRefCB(void* inRefcon, float inValue)
{
	g_kPIDAlt = inValue;
}

float RegkPIDAltDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Vertical/Altitude/kPIDAlt");
	}

	return 0;  // Flight loop is called only once!
}
// / K PID ALT

// D PID ALT
float GetdPIDAltDataRefCB(void* inRefcon)
{
	return g_dPIDAlt;
}

void SetdPIDAltDataRefCB(void* inRefcon, float inValue)
{
	g_dPIDAlt = inValue;
}

float RegdPIDAltDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Vertical/Altitude/dPIDAlt");
	}

	return 0;  // Flight loop is called only once!
}
// / D PID ALT

// AP VS
float GetAPVSDataRefCB(void* inRefcon)
{
	return g_apVS;
}

void SetAPVSDataRefCB(void* inRefcon, float inValue)
{
	g_apVS = inValue;
}

float RegAPVSDataRefInDRE(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Vertical/VerticalSpeed");
	}

	return 0;  // Flight loop is called only once!
}
// / AP VS

// ETAPAS INTERCEPTACION VOR
float RegEtapaInterceptacionVOR(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Lateral/EtapaVOR");
	}

	return 0;  // Flight loop is called only once!
}

int GetEtapaInterceptacionVOR(void* inRefcon)
{
	return g_etapaVOR;
}

void SetEtapaInterceptacionVOR(void* inRefcon, int inValue)
{
	g_etapaVOR = inValue;
}
// / ETAPAS INTERCEPTACION VOR

//ESTADO LOCALIZADOR
float RegEstadoLOC(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (PluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"CUSTOM/AP/Lateral/EstadoLOC");
	}

	return 0;  // Flight loop is called only once!
}


int GetEstadoLOC(void* inRefcon)
{
	return g_estadoLOC;
}

void SetEstadoLOC(void* inRefcon, int inValue)
{
	g_estadoLOC = inValue;
}
// / ESTADO LOCALIZADOR