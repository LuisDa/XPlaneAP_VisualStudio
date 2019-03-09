
/*
 * HellWorld.c
 * 
 * This plugin implements the canonical first program.  In this case, we will 
 * create a window that has the text hello-world in it.  As an added bonus
 * the  text will change to 'This is a plugin' while the mouse is held down
 * in the window.  
 * 
 * This plugin demonstrates creating a window and writing mouse and drawing
 * callbacks for that window.
 * 
 */
#define XPLM200

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"
#include "XPLMScenery.h"

#include "Pid.h"
#include "datarefs.h"
#include "SerialUtil.h"

//SERIAL: http://mitaivision.blogspot.com/2014/03/this-is-tutorial-forthose-who-have.html

//Ver este ejemplo para más información sobre añadir DataRef: http://www.xsquawkbox.net/xpsdk/mediawiki/Register_Custom_DataRef_in_DRE

/*
 * Global Variables.  We will store our single window globally.  We also record
 * whether the mouse is down from our mouse handler.  The drawing handler looks
 * at this information and draws the appropriate display.
 * 
 */

//#define MSG_ADD_DATAREF 0x01000000           //  Add dataref to DRE message

static XPLMWindowID	gWindow = NULL;
static int				gClicked = 0;
static int gCounter = 0;

static float time, time0 = 0;
static float g_prevAlt = 0;
static float g_prevVS = 0;
static float g_prevSpeed = 0;
static float g_prevSpeedTrend = 0;
static float g_speedError = 0;

//Estado de interceptación del VOR:
// 0 -> No hay VOR cercano
// 1 -> La distancia al VOR es inferior a 10 NM y la needle está muy lejos
// 2 -> La distancia al VOR es inferior a 2 NM y la needle sigue muy lejos. Comenzamos a virar
// 3 -> Virando, la needle ya está cerca, el PID coge el control
// Si salimos del modo lateral LOC VOR, pasamos nuevamente a 0
static int g_VORInctpSts = 0;

//Variables asociadas a DataRefs

static PID gCtrlHeading(0.1, 0, 0);
static PID gCtrlAltitude(0.1, 0, 0);
static PID gCtrlSpeedPitch(0.1, 0, 0);
static PID gCtrlVerticalSpeed(0.1, 0, 0);
static PID gCtrlGlideSlope(0.1, 0, 0);
static PID gCtrlVOR_LOC(0.1, 0, 0);

//static SerialUtil* serialUtil = NULL;
static Serial* serial = NULL;

static bool gLocInterceptado = false; //Para los localizadores de los ILS

static void MyDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon);    

static void MyHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus);    

static int MyHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon);    

//Añadimos esta función callback para respuesta a la rueda
static int MyHandleMouseWheelCallback(
									XPLMWindowID         inWindowID,
									int                  x,
									int                  y,
									int                  wheel,
									int                  clicks,
									void *               inRefcon);


//Funciones auxiliares
static void AP_ControlHeading(void);
static float getHeadingIncrement(float hdg_final, float hdg_inicial);
static float getTargetHeading(float hdg_inicial, float increment);
static void AP_ControlAltitude(void); //sim/cockpit2/gauges/indicators/altitude_ft_pilot vs sim/cockpit/autopilot/altitude
static void AP_ControlSpeedPitch(void); //Control de velocidad manejando el pitch
static void AP_ControlVerticalSpeed(void); //Control de velocidad vertical manejando el pitch
static void AP_ControlGlideSlope(void); //Control de la pendiente de descenso en aproximación final

static void AP_ControlRadialVOR(void); //Función alternativa para un VOR

/*
 * XPluginStart
 * 
 * Our start routine registers our window and does any other initialization we 
 * must do.
 * 
 */
PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	/* First we must fill in the passed in buffers to describe our
	 * plugin to the plugin-system. */

	strcpy(outName, "HelloWorld");
	strcpy(outSig, "xplanesdk.examples.helloworld");
	strcpy(outDesc, "A plugin that makes a window.");

	/* Now we create a window.  We pass in a rectangle in left, top,
	 * right, bottom screen coordinates.  We pass in three callbacks. */
	 //  Create our custom integer dataref
	setupDataRefs();

	XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"), 0.00025); //Con el Boeing 727, es 0.000035
	

#if 0
	gWindow = XPLMCreateWindow(
					50, 600, 300, 200,			/* Area of the window. */
					1,							/* Start visible. */
					MyDrawWindowCallback,		/* Callbacks */
					MyHandleKeyCallback,
					MyHandleMouseClickCallback,
					NULL);						/* Refcon - not used. */
#endif
					
	XPLMCreateWindow_t windowDataStr;

	windowDataStr.structSize = sizeof(windowDataStr);
	windowDataStr.left = 50;
	windowDataStr.top = 600;
	windowDataStr.right = 300;
	windowDataStr.bottom = 200;
	windowDataStr.visible = 1;
	windowDataStr.drawWindowFunc = MyDrawWindowCallback;
	windowDataStr.handleKeyFunc = MyHandleKeyCallback;
	windowDataStr.handleMouseClickFunc = MyHandleMouseClickCallback;
	windowDataStr.handleMouseWheelFunc = MyHandleMouseWheelCallback;
	windowDataStr.refcon = NULL;
	gWindow = XPLMCreateWindowEx(&windowDataStr);
	
	//serialUtil = new SerialUtil();//("\\\\.\\COM3");
	serial = new Serial("\\\\.\\COM3");

	/* We must return 1 to indicate successful initialization, otherwise we
	 * will not be called back again. */
	 
	return 1;
}

/*
 * XPluginStop
 * 
 * Our cleanup routine deallocates our window.
 * 
 */
PLUGIN_API void	XPluginStop(void)
{
	/*
	XPLMUnregisterDataAccessor(gDataRef_APModoLateral);
	XPLMUnregisterFlightLoopCallback(RegAPLateralModeDataRefInDRE, NULL);	 //  Don't forget to unload this callback.  
	*/
	tearDownDataRefs();

	XPLMDestroyWindow(gWindow);
}

/*
 * XPluginDisable
 * 
 * We do not need to do anything when we are disabled, but we must provide the handler.
 * 
 */
PLUGIN_API void XPluginDisable(void)
{
}

/*
 * XPluginEnable.
 * 
 * We don't do any enable-specific initialization, but we must return 1 to indicate
 * that we may be enabled at this time.
 * 
 */
PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

/*
 * XPluginReceiveMessage
 * 
 * We don't have to do anything in our receive message handler, but we must provide one.
 * 
 */
PLUGIN_API void XPluginReceiveMessage(
					XPLMPluginID	inFromWho,
					int				inMessage,
					void *			inParam)
{
}

/*
 * MyDrawingWindowCallback
 * 
 * This callback does the work of drawing our window once per sim cycle each time
 * it is needed.  It dynamically changes the text depending on the saved mouse
 * status.  Note that we don't have to tell X-Plane to redraw us when our text
 * changes; we are redrawn by the sim continuously.
 * 
 */
void MyDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon)
{
	int		left, top, right, bottom;
	float	color[] = { 1.0, 1.0, 1.0 }; 	/* RGB White */
	
	/* First we get the location of the window passed in to us. */
	XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
	
	/* We now use an XPLMGraphics routine to draw a translucent dark
	 * rectangle that is our window's shape. */
	XPLMDrawTranslucentDarkBox(left, top, right, bottom);

	/* Finally we draw the text into the window, also using XPLMGraphics
	 * routines.  The NULL indicates no word wrapping. */
	XPLMDrawString(color, left + 5, top - 20, 
		(char*)(gClicked ? "No me interrumpas" : "Ola ke ase"), NULL, xplmFont_Basic);

	//XPLMDrawString(color, left + 5, top - 40, "", NULL, xplmFont_Basic);
	
	time = XPLMGetDataf(XPLMFindDataRef("sim/time/total_flight_time_sec"));

	if (XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/AP_Prendido")) == 1)
	{
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_roll"), 1);		
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_ptch"), 1);
		XPLMSetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode"), 2);
	}
	else
	{
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_roll"), 0);
		XPLMSetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_ptch"), 0);
		XPLMSetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode"), 1);
	}


	//XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 6);
	int ap_modo_vertical = XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo")); //1=modo pitch-speed, 2=modo pitch-VS
	int ap_modo_lateral = XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/Modo"));

	//Detectar modos LOC (VOR e ILS) y G/S (sólo ILS)
	bool nav1_sintonizada = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/radios/nav1_fromto")) != 0); //Hemos visto que si la radioayuda se recibe, este DataRef es distinto de cero (bueno, no comprobado con ILS, sólo con VOR).

	//Si sintonizamos una radioayuda, daremos prioridad al ILS frente al VOR: comprobaremos primero si está el ILS LOC armado, y de lo contrario, miraremos el VOR LOC armado.
	//El piloto deberá estar atento, o bien el FMS que se implemente sobre este piloto automático, tenerlo en cuenta. Por ejemplo, si el piloto desea usar un VOR para aproximarse a un aeropuerto y finalmente hacer la aproximación final con ILS,
	//deberá sintonizar primero el VOR y dejar la frecuencia ILS en stand-by, y cuando ya se acerque, conmutar dichas frecuencias y continuar con ILS hasta tomar tierra.
	bool ils_loc_armado = (XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/ILS_LOC_Armado")) == 1);
	bool ils_gs_armado = (XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/ILS_GS_Armado")) == 1);
	bool vor_loc_armado = (XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/VOR_LOC_Armado")) == 1);
	float nav_hdef_dot = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_hdef_dot"));
	float nav_vdef_dot = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_vdef_dot"));

	if (nav1_sintonizada)
	{
		if (ils_loc_armado)
		{
			if ((fabs(nav_hdef_dot) < 2) && (ap_modo_lateral != 3))
			{
				XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/Modo"), 3);
				ap_modo_lateral = 3;
			}
		} 
		else if (vor_loc_armado)
		{
			if ((fabs(nav_hdef_dot) < 2) && (ap_modo_lateral != 2))
			{
				XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/Modo"), 2);
				ap_modo_lateral = 2;
			}
		}

		if (ils_gs_armado && ap_modo_lateral == 3)
		{
			if ((fabs(nav_vdef_dot) < 1.5) && (ap_modo_vertical != 3))
			{
				XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 3);
				ap_modo_vertical = 3;
			}
		}
	}

	if (ap_modo_lateral <= 8) AP_ControlHeading();
	else if (ap_modo_lateral == 9) AP_ControlRadialVOR(); //Volar un VOR con esta función específica

	float current_speed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot"));
	float ap_airspeed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/airspeed"));
	
	if (time - time0 > 0) g_prevSpeedTrend = 10 * ((current_speed - g_prevSpeed) / (time - time0));
	g_speedError = ap_airspeed - (current_speed + g_prevSpeedTrend);

	if (ap_modo_vertical != 20)
	{
		if (ap_modo_vertical < 3) AP_ControlAltitude(); //Si no estamos en G/S, mantenemos altitud

		//IDEA: Control de velocidad, variando con un PID la V/S, si el modo es de V/S, ésta se coge del selector.
		//if (ap_modo_vertical == 1) AP_ControlSpeedPitch();
		//else AP_ControlVerticalSpeed();
		if (ap_modo_vertical == 1) AP_ControlSpeedPitch();
		else if ((ap_modo_vertical == 3) || (ap_modo_vertical == 13) || (ap_modo_vertical == 14)) AP_ControlGlideSlope();

		if (ap_modo_vertical != 13) AP_ControlVerticalSpeed();
	}

	g_prevSpeed = current_speed;

	time0 = time;	
	
	//if (serialUtil != NULL)
	//{
	try {
		//serialUtil = new SerialUtil();//("\\\\.\\COM3");

		//if (serialUtil != NULL)
		if (serial != NULL)
		{
			//while (SerialUtil::SP->IsConnected())
			//if (SerialUtil::SP->IsConnected())
			if (serial->IsConnected())
			{
				
				//string data_read;
				//data_read = serialUtil->read();
				char data_read_chr[4] = {0, 0, 0, 0};

				int num_bytes = serial->ReadData(data_read_chr, 4);

				//if (!data_read.empty())
				if (num_bytes == 4)
				{
					//const char* data_read_chr = data_read.c_str();

					//float ap_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"));
					float current_thr_vect_ratio = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/controls/thrust_vector_ratio"));

					//if (data_read_chr[2] == 0xAB)
					//{
						//if (data_read.at(0) == 'a')
						if (data_read_chr[0] == 'a')
						{
							//ap_heading = ap_heading + 10;
							//if (ap_heading > 359.5) ap_heading = 0;
							//else if (ap_heading < 0) ap_heading = 359;
							////XPLMSetDataf(XPLMFindDataRef("sim/cockpit2/controls/thrust_vector_ratio"), current_thr_vect_ratio - 0.1);
							//XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), ap_heading);

							float ap_VS = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));
							ap_VS -= 100;
							XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"), ap_VS);
						}
						else if (data_read_chr[0] == 'b')
						{
							//ap_heading = ap_heading - 10;
							//if (ap_heading > 359.5) ap_heading = 0;
							//else if (ap_heading < 0) ap_heading = 359;
							//XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), ap_heading);
							////XPLMSetDataf(XPLMFindDataRef("sim/cockpit2/controls/thrust_vector_ratio"), current_thr_vect_ratio + 0.1);

							float ap_VS = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));
							ap_VS += 100;
							XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"), ap_VS);
						}

						unsigned int pote_vectorial = (unsigned int)data_read_chr[1];

						//Ñapa para que siempre nos dé un valor positivo (pero como solución es un zurullo de vaca)
						//Cuando el byte pasa de 127, por razones que ahora desconocemos, pasa a ponerse un valor negativo. Hacemos una conversión para que haya continuidad, y antes parece pasar por el valor cero
						//Definimos, por tanto, una función a tramos: 
						// - Si nos da menor o igual a 1127, lo dejamos como está
						// - Si nos da igual a 1000, lo ponemos a 1128
						// - Y si nos da INFERIOR a 1000, es que el valor original es negativo, y haremos que dé entre 1128 y 1255
						if (pote_vectorial > 127) pote_vectorial += 254;

						//pote_vectorial += 1000;
						/*
						if (pote_vectorial < 0)
						{
							pote_vectorial += 255;
						}
						*/

						XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/EstadoLOC"), pote_vectorial);

						float palanca_vectorial = (1.0*pote_vectorial) / 255;

						//float palanca_vectorial = ((float)data_read_chr[1]) / 255.0;
						XPLMSetDataf(XPLMFindDataRef("sim/cockpit2/controls/thrust_vector_ratio"), palanca_vectorial);
						
					//}
				}		

				PurgeComm(serial->getHandle(), PURGE_RXCLEAR);
			}

			//delete serialUtil;
			//serialUtil = NULL;
		}
	}
	catch (string s)
	{

	}


	//}
	

	/*
	float ap_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"));
	ap_heading = ap_heading + clicks;
	if (ap_heading > 359.5) ap_heading = 0;
	else if (ap_heading < 0) ap_heading = 359;
	XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), ap_heading);
	*/
}                                   

/*
 * MyHandleKeyCallback
 * 
 * Our key handling callback does nothing in this plugin.  This is ok; 
 * we simply don't use keyboard input.
 * 
 */
void MyHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus)
{
	if ((inKey == XPLM_KEY_UP) || (inVirtualKey == XPLM_VK_U))
	{
		float ap_VS = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));
		ap_VS += 100;
		XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"), ap_VS);
	}
	else if ((inKey == XPLM_KEY_DOWN) || (inVirtualKey == XPLM_VK_D))
	{
		float ap_VS = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));
		ap_VS -= 100;
		XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"), ap_VS);
	}
		
}                                   

/*
 * MyHandleMouseClickCallback
 * 
 * Our mouse click callback toggles the status of our mouse variable 
 * as the mouse is clicked.  We then update our text on the next sim 
 * cycle.
 * 
 */
int MyHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon)
{
	/* If we get a down or up, toggle our status click.  We will
	 * never get a down without an up if we accept the down. */
	if ((inMouse == xplm_MouseDown) || (inMouse == xplm_MouseUp))
	{
		gClicked = 1 - gClicked;
	}

	if (inMouse == xplm_MouseDown)
	{
		//Prueba sencilla: con click del ratón, cambiamos estado del paracaídas de los aviones supersónicos
		int parachute_sts = XPLMGetDatai(XPLMFindDataRef("sim/cockpit/switches/parachute_on"));
		XPLMSetDatai(XPLMFindDataRef("sim/cockpit/switches/parachute_on"), (parachute_sts == 1) ? 0 : 1);
	}
	
	/* Returning 1 tells X-Plane that we 'accepted' the click; otherwise
	 * it would be passed to the next window behind us.  If we accept
	 * the click we get mouse moved and mouse up callbacks, if we don't
	 * we do not get any more callbacks.  It is worth noting that we 
	 * will receive mouse moved and mouse up even if the mouse is dragged
	 * out of our window's box as long as the click started in our window's 
	 * box. */
	return 1;
}                                      


int MyHandleMouseWheelCallback(
	XPLMWindowID         inWindowID,
	int                  x,
	int                  y,
	int                  wheel,
	int                  clicks,
	void *               inRefcon)
{
	/*
	float ap_airspeed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/airspeed"));
	ap_airspeed = ap_airspeed + clicks;
	if (ap_airspeed < 0) ap_airspeed = 0;
	XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/airspeed"), ap_airspeed);
	*/
	float ap_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"));
	ap_heading = ap_heading + clicks;
	if (ap_heading > 359.5) ap_heading = 0;
	else if (ap_heading < 0) ap_heading = 359;
	XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), ap_heading);

	return 1;
}

void AP_ControlHeading()
{
	//time = XPLMGetDataf(XPLMFindDataRef("sim/time/total_flight_time_secs"));

	//Control de heading: sólo con override_flight director
	bool override_fdRoll = (XPLMGetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_roll")) == 1);
	bool autopilot_prendido = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")) == 2);

	if (autopilot_prendido && override_fdRoll)
	{
		//float ap_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"));
		float ap_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading_mag"));
		float current_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/heading_AHARS_deg_mag_pilot"));
		float nav_course = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_obs_degt")); //Para navegación VOR, curso o radial a interceptar
		float ils_course = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_course_degm")); //Para un ILS, curso de la pista donde aterrizar
		float nav_hdef_dot = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_hdef_dot")); //Para navegación VOR o ILS, indicador de desviación con respecto al radial a interceptar
		int modo_lateral = XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/Modo"));

		//Probamos el caso "normal" (sin pasar por heading=360 grados)
		float roll = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_roll"));
		//float hdg_diff = ap_heading - current_heading;

		float hdg_diff = getHeadingIncrement(ap_heading, current_heading);
		float crs_inc = getHeadingIncrement(nav_course, current_heading);


		if (modo_lateral == 2) //VOR //XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"));
		{
			XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"), crs_inc);

			if (fabs(crs_inc) < 60)
			{
				if (fabs(nav_hdef_dot) < 0.6)
				{
					current_heading = getTargetHeading(nav_course, 50 * nav_hdef_dot);
					XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
					XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"), 1);
				}
				else
				{
					if (nav_hdef_dot > 0) current_heading = getTargetHeading(nav_course, 30);
					else current_heading = getTargetHeading(nav_course, -30);

					XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
					XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"), 2);
				}
			}
			else //Si la diferencia en valor absoluto es superior a 60º, virar por el camino más corto
			{
				if (crs_inc > 0) //Viraje a la derecha
				{
					current_heading = getTargetHeading(current_heading, crs_inc + 30);
					XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"), 3);
				}
				else //Viraje a la izquierda
				{
					current_heading = getTargetHeading(current_heading, crs_inc - 30);
					XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"), 4);
				}
			}
			/*
			if (((fabs(crs_inc) < 30)) && (fabs(nav_hdef_dot) < 0.6))
			{
				current_heading = getTargetHeading(nav_course, 50 * nav_hdef_dot);
				XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
			}
			else if (fabs(crs_inc) < 60) //De momento contemplamos únicamente el caso de estar dentro de un abanico de 60º en torno al radial
			{
				if (nav_hdef_dot > 0) current_heading = getTargetHeading(nav_course, 30);
				else current_heading = getTargetHeading(nav_course, -30);

				XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
			}
			*/
		}
		else if (modo_lateral == 3) //ILS
		{
			current_heading = getTargetHeading(ils_course, 30 * nav_hdef_dot);
			XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
		}

/*
		if ( ((modo_lateral == 2) && ((fabs(crs_inc) < 60) && (fabs(nav_hdef_dot) < 2))) //Navegación lateral con VOR
			|| ((modo_lateral == 3) && (fabs(nav_hdef_dot) < 2))) //Navegación lateral con ILS
		{
			//Controlaremos el vuelo del VOR variando el rumbo //getTargetHeading(inicial, incr)
			if (modo_lateral == 2) //VOR
			{
				current_heading = getTargetHeading(nav_course, 30 * nav_hdef_dot);
				XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
			}
			else if (modo_lateral == 3) //ILS
			{
				current_heading = getTargetHeading(ils_course, 30 * nav_hdef_dot);
				XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/heading"), current_heading);
			}
		}
*/		

		//if (getHeadingIncrement(ap_heading, current_heading) < -30) roll = -30;
		//else if (getHeadingIncrement(ap_heading, current_heading) > 30) roll = 30;
		if (getHeadingIncrement(ap_heading, current_heading) < -15) roll = -30;
		else if (getHeadingIncrement(ap_heading, current_heading) > 15) roll = 30;
		else
		{
			gCtrlHeading.setPropGain(2);

			if (time - time0 > 0)
			{
				roll = gCtrlHeading.getOutput(hdg_diff, time - time0);
			}

			if (roll > 30) roll = 30;
			else if (roll < -30) roll = -30;
		}

		XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_roll"), roll);
	}
	
	//time0 = time;
}

void AP_ControlRadialVOR(void)
{
	bool override_fdRoll = (XPLMGetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_roll")) == 1);
	bool autopilot_prendido = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")) == 2);
	
	if (autopilot_prendido && override_fdRoll)
	{
		float current_heading = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/heading_AHARS_deg_mag_pilot"));
		float nav_course = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_obs_degt")); //Para navegación VOR, curso o radial a interceptar
		float nav_hdef_dot = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_hdef_dot"));
		float nav_dme = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_dme_dist_m"));
		float crs_inc = getHeadingIncrement(nav_course, current_heading);
		float roll = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_roll"));

		//Habrá distintos modos de interceptar. Según nos aproximemos al radial y con qué ángulo, habrá que ver la variación de la aguja y con eso controlar el alabeo
		//Se podrá sacar un "needle trend" y en función del mismo, poner un alabeo mayor o menor (máximo alabeo, 30º, con eso también podríamos ver la distancia de anticipación).
		//Cuando estemos ya próximos a interceptar, activamos el PID que controla el alabeo en función de la aguja. Se podrá también tener en cuenta la diferencia entre el radial y el rumbo actual => posiblemente se necesite una máquina de estados GRAFCET.

		//Definimos etapa del GRAFCET:
		// 0 -> No hay VOR cercano
		// 1 -> La distancia al VOR es inferior a 10 NM y la needle está muy lejos
		// 2 -> La distancia al VOR es inferior a 2 NM y la needle sigue muy lejos. Comenzamos a virar
		// 3 -> Virando, la needle ya está cerca, el PID coge el control
		// Si salimos del modo lateral LOC VOR, pasamos nuevamente a 0
		/*
		if ((g_VORInctpSts == 0) && (nav_dme <= 10) && (fabs(nav_hdef_dot) > 1.5)) g_VORInctpSts = 1; // Nos acercamos al VOR y la needle queda muy lejos
		else if ((g_VORInctpSts == 1) && (nav_dme <= 2) && (fabs(nav_hdef_dot) > 1.5)) g_VORInctpSts = 2; //Estamos a menos de 2 NM del VOR y la needle sigue quedando lejos => Toca hacer el viraje
		else if ((g_VORInctpSts == 2) && (fabs(nav_hdef_dot) < 1)) g_VORInctpSts = 3; //Needle ya cerca e interceptada, damos el control al PID
		*/

		//Algoritmo alternativo: esperar a interceptar y ahí ya que entre en juego el PID o el alabeo controlado. Probamos con delta (diferencia entre rumbo y curso) no muy grandes
		if ((g_VORInctpSts == 0) && (fabs(nav_hdef_dot) < 2.2)) g_VORInctpSts = 1; //Comenzamos a acercarnos al cero de la aguja -> Tocará alabear 30º a izquierda o derecha según corresponda
		else if ((g_VORInctpSts == 1) && (fabs(crs_inc) < 5) && (fabs(nav_hdef_dot) < 1.5)) g_VORInctpSts = 2; //

		XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Lateral/EtapaVOR"), g_VORInctpSts);

		//gCtrlVOR_LOC
		float kPID_Vor = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"));
		float dPID_Vor = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"));
		
		if (g_VORInctpSts == 1)
		{
			if (crs_inc < 0) roll = -30; //El curso queda a la izquierda de nuestro rumbo -> Virar a la izquierda
			else if (crs_inc > 0) roll = 30; //El curso queda a la derecha de nuestro rumbo -> Virar a la derecha
		}
		else if (g_VORInctpSts == 2)
		{
			gCtrlVOR_LOC.setPropGain(kPID_Vor);
			gCtrlVOR_LOC.setDerGain(dPID_Vor);
			if (time - time0 > 0)
			{
				roll = gCtrlVOR_LOC.getOutput(nav_hdef_dot, time - time0);
			}

			if (roll > 20 * nav_hdef_dot) roll = 20 * nav_hdef_dot;
			else if (roll < -20 * nav_hdef_dot) roll = -20 * nav_hdef_dot;

			//if (roll > 10) roll = 10;
			//else if (roll < -10) roll = -10;
		}

		/*
		if (g_VORInctpSts == 1)
		{
			gCtrlVOR_LOC.setPropGain(kPID_Vor);
			gCtrlVOR_LOC.setDerGain(dPID_Vor);
			roll = gCtrlVOR_LOC.getOutput(nav_hdef_dot, time - time0);

			if (roll > 30) roll = 30;
			else if (roll < -30) roll = -30;
		}
		*/


		XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_roll"), roll);
	}
}

void AP_ControlAltitude()
{
	bool override_fdPitch = (XPLMGetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_ptch")) == 1);
	bool autopilot_prendido = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")) == 2);
	float current_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot"));
	//XPLMSetDataf(XPLMFindDataRef("CUSTOM/Flight/VertSpeed"),  time - time0);


	if ((time - time0 > 0) && (current_altitude - g_prevAlt != 0))
	{
		g_vertSpeed = (current_altitude - g_prevAlt) / (1.0*(time - time0));
		XPLMSetDataf(XPLMFindDataRef("CUSTOM/Flight/VertSpeed"), 60*g_vertSpeed);
	}

	
	if (autopilot_prendido && override_fdPitch)
	{
		float ap_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/altitude"));
		//float current_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot"));

		float pitch = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"));

		//float alt_diff = ap_altitude - (current_altitude + g_vertSpeed*10);
		float alt_diff = ap_altitude - current_altitude;

		//Control de pitch de momento, básico
		if ((alt_diff < -100) && (alt_diff > -500)) pitch = -2.5;
		else if (alt_diff > 100 && (alt_diff < 500)) pitch = 8.5;
		//else if (alt_diff > 500) pitch = (alt_diff - 50) / 150 + 2;
		//else if (alt_diff < -500) pitch = -((alt_diff - 50) / 150 + 2);
		else if ((alt_diff > -100) && (alt_diff < 100))
		{
			//gCtrlAltitude.setPropGain(0.1); // CUSTOM/AP/Vertical/Altitude/kPIDAlt
			//Valores óptimos: King Air -> 0.0001 (con los gases bien regulados, si no, a máximo gas, oscila). En descenso oscila que es un canteo horroroso. Lo siguiente será implementar control de pitch-velocidad
			g_kPIDAlt = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"));
			gCtrlAltitude.setPropGain(0.00002/*g_kPIDAlt*/); //0.00002

			if (time - time0 > 0)
			{
				pitch += gCtrlAltitude.getOutput(ap_altitude - (current_altitude + g_vertSpeed * 1), time - time0);
			}


			if (pitch > 8.5) pitch = 8.5;
			else if (pitch < -2.5) pitch = -2.5;					

			//XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"), pitch);
		}				
		

		g_prevAlt = current_altitude;

		XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"), pitch);
	}

	g_prevAlt = current_altitude;
}

void AP_ControlVerticalSpeed(void)
{
	//gCtrlVerticalSpeed
	float currVS = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm"));	
	float ap_VS = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));

	bool override_fdPitch = (XPLMGetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_ptch")) == 1);
	bool autopilot_prendido = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")) == 2);
	float current_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot"));
	float ap_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/altitude"));
	float pitch = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"));
	int ap_modo_vertical = XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo")); //1=modo pitch-speed, 2=modo pitch-VS


	//gCtrlVerticalSpeed.setPropGain(1);
	
	if (autopilot_prendido && override_fdPitch)
	{
		if (((ap_altitude - current_altitude > 500) || (ap_altitude - current_altitude < -500)) || (ap_modo_vertical >= 3))
		{

			g_kPIDAlt = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"));
			g_dPIDAlt = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"));

			if (ap_VS >= currVS)
			{
				if (ap_VS - currVS > 300)
				{
					//XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 1);
					pitch += 0.02;
				}
				/*
				else if (ap_VS - currVS > 100)
				{
					XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 2);
					pitch += 0.001;
				}
				*/
				else
				{
					//XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 3);
					//gCtrlVerticalSpeed.setPropGain(g_kPIDAlt); //Con la Baron, 0.00001
					//gCtrlVerticalSpeed.setDerGain(g_dPIDAlt);
					gCtrlVerticalSpeed.setDerGain(0.000002); //Beechcraft Baron
					gCtrlVerticalSpeed.setPropGain(0.000025);

					//float vs_= (currVS - g_prevVS) / (time - time0);
					if (time - time0 > 0)
					{
						pitch += gCtrlVerticalSpeed.getOutput(ap_VS - currVS, time - time0);
					}

					if (pitch > 15) pitch = 15;
				}
			}
			else
			{
				if (currVS - ap_VS > 300)
				{
					//XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 4);
					pitch -= 0.02;
				}
				/*
				else if (currVS - ap_VS > 100)
				{
					XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 5);
					pitch -= 0.001;
				}
				*/
				else
				{
					//XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 6);
					//gCtrlVerticalSpeed.setPropGain(g_kPIDAlt); //Con la Baron, 0.00001
					//gCtrlVerticalSpeed.setDerGain(g_dPIDAlt);
					gCtrlVerticalSpeed.setDerGain(0.000002);
					gCtrlVerticalSpeed.setPropGain(0.000025);
					//gCtrlVerticalSpeed.setDerGain(g_dPIDAlt);
					//float vs_trend = (currVS - g_prevVS) / (time - time0);
					if (time - time0 > 0)
					{
						pitch += gCtrlVerticalSpeed.getOutput(ap_VS - currVS, time - time0);
					}
					if (pitch < -15) pitch = -15;
				} 
			}

			g_prevVS = currVS;
			XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"), pitch);

		}
	}

}

void AP_ControlSpeedPitch(void)
{
	bool override_fdPitch = (XPLMGetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_ptch")) == 1);
	bool autopilot_prendido = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")) == 2);
	float current_speed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot"));
	float ap_airspeed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/airspeed"));

	float current_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot"));
	float ap_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/altitude"));
	float pitch = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"));
	
	//float speed_trend = 0;//(current_speed - g_prevSpeed) / (time - time0);
	//if (time - time0 > 0) g_prevSpeedTrend = 10*((current_speed - g_prevSpeed) / (time - time0));

	//float ap_VS = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));

	if (autopilot_prendido && override_fdPitch)
	{
		g_kPIDAlt = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt")); //0.1
		g_dPIDAlt = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt")); //0.01
		float current_vs = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));

		//float spd_error = ap_airspeed - (current_speed + g_prevSpeedTrend); //Si es positivo, bajar morro, si es negativo, subir morro
		//g_speedError = ap_airspeed - (current_speed + g_prevSpeedTrend);
		//gCtrlSpeedPitch.setPropGain(g_kPIDAlt);
		//gCtrlSpeedPitch.setDerGain(g_dPIDAlt);
		gCtrlSpeedPitch.setPropGain(0.1);
		gCtrlSpeedPitch.setDerGain(0.01);


		float vs_act = 0;

		if (ap_altitude - current_altitude > 500) //Ascenso
		{	
			
			if (time - time0 > 0)
			{
				//vs_act = current_vs - gCtrlSpeedPitch.getOutput(spd_error, time - time0);
				vs_act = current_vs - gCtrlSpeedPitch.getOutput(g_speedError, time - time0);
				
				if (vs_act > 3000) vs_act = 3000;
				else if (vs_act < 200) vs_act = 200;
			}			
		}
		else if (ap_altitude - current_altitude < -500) //Descenso
		{		
			if (time - time0 > 0)
			{
				//vs_act = current_vs - gCtrlSpeedPitch.getOutput(spd_error, time - time0);
				vs_act = current_vs - gCtrlSpeedPitch.getOutput(g_speedError, time - time0);
				
				if (vs_act > -200) vs_act = -200;
				else if (vs_act < -3500) vs_act = -3500;
			}			
		}

		XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"), vs_act);		
	}

	g_prevSpeed = current_speed;
}

void AP_ControlGlideSlope(void)
{
	bool override_fdPitch = (XPLMGetDatai(XPLMFindDataRef("sim/operation/override/override_flightdir_ptch")) == 1);
	bool autopilot_prendido = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode")) == 2);
	float current_speed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot"));
	float ap_airspeed = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/airspeed"));

	float current_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot"));
	float ap_altitude = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/altitude"));
	float pitch = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"));
	float rad_alt = XPLMGetDataf(XPLMFindDataRef("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot"));

	bool ils_has_dme = (XPLMGetDatai(XPLMFindDataRef("sim/cockpit/radios/nav1_has_dme")) == 1);
	float ils_dme_dist = XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_dme_dist_m"));

	int ap_modo_vertical = XPLMGetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo")); //3=modo G/S armado, 13= modo G/S activo => Se presupone interceptación de la senda por debajo

	float nav_vdef_dot = 100*XPLMGetDataf(XPLMFindDataRef("sim/cockpit/radios/nav1_vdef_dot"));

	float vs_act = 0;

	if (autopilot_prendido && override_fdPitch)
	{
		if (ap_modo_vertical == 3)
		{
			vs_act = 0;

			//Chequeamos la needle (debe ser negativa para interceptación por debajo)
			//if ((nav_vdef_dot > -0.4) && (nav_vdef_dot < -0.1))
			if ((nav_vdef_dot > -40) && (nav_vdef_dot < -20))
			{
				vs_act = -300;
			}			
			else if (nav_vdef_dot >= -20)
			{
				vs_act = -500;
				//XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 13);
				XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 14);
			}			
		}
		else if (ap_modo_vertical == 14) //Variar la V/S entre -500 y -800 fpm
		{

			float der_gain = 0.08;//XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"));
			float prop_gain = 0.005; //XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt"));

			//prop_gain = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/kPIDAlt")); //0.1
			//der_gain = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/Altitude/dPIDAlt"));

			/*
			if (rad_alt < 250)
			{
				der_gain = 0.008;
				prop_gain = 0.0005;
			}*/

			//Cuando estemos cerca, modificamos esto para que no oscile tanto (el estrechamiento del cono del ILS hace que varíe más sensiblemente el valor de la aguja de G/S)
			if (ils_has_dme && (fabs(ils_dme_dist) < 5))
			{
				//der_gain = 8;
				//prop_gain = 0.5;
				/*
				if (nav_vdef_dot > -0.1)
				{
					vs_act -= 10;
				}
				else if (nav_vdef_dot < -0.2)
				{
					vs_act += 10;
				}
				else //Aplicamos esta recta: -0.1 -> -600 fpm / -0.2 -> -800 fpm
				{
					vs_act = 2000 * (nav_vdef_dot + 0.1) - 600;
				}

				if (vs_act > -600) vs_act = -600;
				else if (vs_act < -800) vs_act = -800;
				*/
			}

			gCtrlGlideSlope.setPropGain(prop_gain);
			gCtrlGlideSlope.setDerGain(der_gain);

			vs_act = XPLMGetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"));

			if (time - time0 > 0)
			{
				float increment = gCtrlGlideSlope.getOutput(nav_vdef_dot, time - time0);
				vs_act -= increment;
			}			

			if (vs_act > -100) vs_act = -100;
			else if (vs_act < -1000) vs_act = -1000;



			if (rad_alt < 30) XPLMSetDatai(XPLMFindDataRef("CUSTOM/AP/Vertical/Modo"), 13);			
		}
		else if (ap_modo_vertical == 13) //Usamos este modo para el último tramo, de toma de pista.
		{
			XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"), 3.5);
		}

		XPLMSetDataf(XPLMFindDataRef("CUSTOM/AP/Vertical/VerticalSpeed"), vs_act);
		//XPLMSetDataf(XPLMFindDataRef("sim/cockpit/autopilot/flight_director_pitch"), pitch);
	}
}


/*
Control de rumbo.

- Puede darse el caso de que el rumbo que pongamos esté, por el camino más corto, pasando por 0º. Eso sucederá si al calcular la diferencia nos sale con valor absoluto superior a 180º. En ese caso:

	- Si el rumbo objetivo es mayor de 180 grados, la ecuación a aplicar sería hdg_inc = (hdg_final - 360) - hdg_inicial; //VIRAJE A IZQUIERDAS PASANDO POR 360. Eso sucede si hdg_ini < 180, hdf_fin > 180 Y la diferencia inicial supera 180.
	- Si el rumbo objetivo es inferior a 180 grados, y hdg_ini > 180, aplicamos hdg_inc = hdg_final + (360 - hdg_inicial)


*/
float getHeadingIncrement(float hdg_final, float hdg_inicial)
{
	if (fabs(hdg_final - hdg_inicial) <= 180)
	{
		return hdg_final - hdg_inicial;
	}
	else
	{
		if (hdg_final > 180) return (hdg_final - 360) - hdg_inicial; //Viraje a izquierdas pasando por 360.
		else return hdg_final + (360 - hdg_inicial); //Viraje a derechas pasando por 360.
	}
}


static float getTargetHeading(float hdg_inicial, float increment)
{
	if (hdg_inicial + increment < 0) //Viraje a izquierdas pasando por 360º
	{
		//return 360 - (increment - hdg_inicial);
		return 360 + (hdg_inicial + increment);
	}
	else if (hdg_inicial + increment > 360) //Viraje a derechas pasando por 360º
	{
		return increment - (360 - hdg_inicial);
	}
	else
	{
		return hdg_inicial + increment;
	}
}