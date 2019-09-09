/*
 * Copyright (C) 2014 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#include <python2.7/Python.h>
#include <QtPlugin>
#include <QTimer>
#include "basic_aps_plugin.h"

/*! Duration for which the idle state will be hold before state machine proceeds. */
#define IDLE_TIMEOUT                  (10 * 6000)

/*! Duration to wait for Match_Descr_rsp frames after sending the request. */
#define WAIT_MATCH_DESCR_RESP_TIMEOUT (10 * 1000)
#define WAIT_SIMPLE_DESCR_RESP_TIMEOUT (10 * 1000)
#define WAIT_BTLE_TIMEOUT (10 * 1000)


#define TM_PROFILE_ID    0x0402    /*! TM : Temperature measurement   */
#define ONOFF_PROFILE_ID 0x0006    /*! ONOFF : ONOFF cluster ID       */
#define HA_PROFILE_ID    0x0104    /*! HA : Home Automation           */
#define DST_NODE         0x7dff    /*! hardcode the @ of the sensor   */
#define ACT_NODE         0x3181    /*! hardcode the @ of the actuator */

#define ONOFF_COMMAND_ON  0x01
#define ONOFF_COMMAND_OFF 0x00

#define TM_MIN 1700 /* minimal temperature authorized */
#define TM_MAX 2100 /* threshold to send command off*/


/*! Plugin constructor.
    \param parent - the parent object
 */
BasicApsPlugin::BasicApsPlugin(QObject *parent) :
    QObject(parent)
{
    cmd = false;
    m_btle = 0;
    btle_temperature = -100;
    m_state = StateIdle;
    // keep a pointer to the ApsController
    m_apsCtrl = deCONZ::ApsController::instance();
    DBG_Assert(m_apsCtrl != 0);

    // APSDE-DATA.confirm handler
    connect(m_apsCtrl, SIGNAL(apsdeDataConfirm(const deCONZ::ApsDataConfirm&)),
            this, SLOT(apsdeDataConfirm(const deCONZ::ApsDataConfirm&)));

    // APSDE-DATA.indication handler
    connect(m_apsCtrl, SIGNAL(apsdeDataIndication(const deCONZ::ApsDataIndication&)),
            this, SLOT(apsdeDataIndication(const deCONZ::ApsDataIndication&)));

    // timer used for state changes and timeouts
    m_timer = new QTimer(this);
    m_timer->setSingleShot(true);
    connect(m_timer, SIGNAL(timeout()),
            this, SLOT(timerFired()));

    // start the state machine
    m_timer->start(6000);
}

/*! Deconstructor for plugin.
 */
BasicApsPlugin::~BasicApsPlugin()
{
    m_apsCtrl = 0;
}

/*! APSDE-DATA.indication callback.
    \param ind - the indication primitive
    \note Will be called from the main application for every incoming indication.
    Any filtering for nodes, profiles, clusters must be handled by this plugin.
 */
void BasicApsPlugin::apsdeDataIndication(const deCONZ::ApsDataIndication &ind)
{  
    if (ind.profileId() == ZDP_PROFILE_ID)
    {
        if (ind.clusterId() == ZDP_MATCH_DESCRIPTOR_RSP_CLID)
        {
            handleMatchDescriptorResponse(ind);
        }

        else if (ind.clusterId() == ZDP_SIMPLE_DESCRIPTOR_RSP_CLID)
        {
            handleSimpleDescriptorResponse(ind);
        }
    }
    else if (ind.profileId() == HA_PROFILE_ID)
    { 
	deCONZ::ZclFrame zclFrame;
	{
	    QDataStream stream(ind.asdu());
	    stream.setByteOrder(QDataStream::LittleEndian);
	    zclFrame.readFromStream(stream);

	    handleZCLReadAttributeResponse(ind, zclFrame);
	}

    }
    else if (m_btle == 1)
    {
	handleBTLEResponse();
    }
}

/*! APSDE-DATA.confirm callback.
    \param conf - the confirm primitive
    \note Will be called from the main application for each incoming confirmation,
    even if the APSDE-DATA.request was not issued by this plugin.
 */
void BasicApsPlugin::apsdeDataConfirm(const deCONZ::ApsDataConfirm &conf)
{
    std::list<deCONZ::ApsDataRequest>::iterator i = m_apsReqQueue.begin();
    std::list<deCONZ::ApsDataRequest>::iterator end = m_apsReqQueue.end();

    // search the list of currently active requests
    // and check if the confirmation belongs to one of them
    for (; i != end; ++i)
    {
        if (i->id() == conf.id())
        {
            m_apsReqQueue.erase(i);

            if (conf.status() == deCONZ::ApsSuccessStatus)
            {
                stateMachineEventHandler(EventSendDone);
            }
            else
            {
                DBG_Printf(DBG_INFO, "APS-DATA.confirm failed with status: 0x%02X\n", conf.status());
                stateMachineEventHandler(EventSendFailed);
            }
            return;
        }
    }
}

/*! Handles a match descriptor response.
    \param ind a ZDP Match_Descr_rsp
 */
void BasicApsPlugin::handleMatchDescriptorResponse(const deCONZ::ApsDataIndication &ind)
{

    QDataStream stream(ind.asdu());
    stream.setByteOrder(QDataStream::LittleEndian);

    uint8_t zdpSeq;
    uint8_t status;
    uint16_t nwkAddrOfInterest;
    uint8_t matchLength;
    uint8_t endpoint;

    stream >> zdpSeq;

    // only handle the Match_Descr_rsp which belongs to our request
    if (zdpSeq != m_matchDescrZdpSeq)
    {
        return;
    }

    stream >> status;

    DBG_Printf(DBG_INFO, "received match descriptor response (id: %u) from %s\n", m_matchDescrZdpSeq, qPrintable(ind.srcAddress().toStringExt()));

    if (status == 0x00) // SUCCESS
    {
        stream >> nwkAddrOfInterest;
        stream >> matchLength;

        while (matchLength && !stream.atEnd())
        {
            matchLength--;
            stream >> endpoint;
            DBG_Printf(DBG_INFO, "\tmatch descriptor endpoint: 0x%02X\n", endpoint);
        }

        // done restart state machine
        if (m_state == StateWaitMatchDescriptorResponse)
        {
            setState(StateIdle);
            m_timer->stop();
            m_timer->start(IDLE_TIMEOUT);
        }
    }
}


/*! Handles a simple descriptor response.
    \param ind a ZDP Simple_Descr_rsp
 */
void BasicApsPlugin::handleSimpleDescriptorResponse(const deCONZ::ApsDataIndication &ind)
{

    QDataStream stream(ind.asdu());
    stream.setByteOrder(QDataStream::LittleEndian);

    uint8_t zdpSeq;
    uint8_t status;
    uint16_t nwkAddrOfInterest;
    uint8_t matchLength;
    uint8_t endpoint;

    stream >> zdpSeq;

    // only handle the Match_Descr_rsp which belongs to our request
    if (zdpSeq != m_simpleDescrZdpSeq)
    {
        return;
    }

    stream >> status;

    DBG_Printf(DBG_INFO, "received simple descriptor response (id: %u) from %s\n", m_simpleDescrZdpSeq, qPrintable(ind.srcAddress().toStringExt()));

    if (status == 0x00) // SUCCESS
    {
        stream >> nwkAddrOfInterest;
        stream >> matchLength;

        while (matchLength && !stream.atEnd())
        {
            matchLength--;
            stream >> endpoint;
            DBG_Printf(DBG_INFO, "\tsimple descriptor endpoint: 0x%02X\n", endpoint);
        }

        // done restart state machine
        if (m_state == StateWaitSimpleDescriptorResponse)
        {
            setState(StateIdle);
            m_timer->stop();
            m_timer->start(IDLE_TIMEOUT);
        }
    }
}

/*! Handler for simple timeout timer.
 */
void BasicApsPlugin::timerFired()
{
    stateMachineEventHandler(EventTimeout);
}


/*! Test to send a ZCL read attribut request
    \return true if request was added to queue
 */
bool BasicApsPlugin::sendZCLReadAttributeRequest(quint16 dstNode, quint8 cmdId, quint16 clusterID)
{
    DBG_Assert(m_state == StateIdle);

    if (m_apsCtrl->networkState() != deCONZ::InNetwork)
    {
        return false;
    }

    deCONZ::ApsDataRequest apsReq;

    // set destination addressing
    apsReq.setDstAddressMode(deCONZ::ApsNwkAddress);
    apsReq.dstAddress().setNwk(dstNode);
    apsReq.setDstEndpoint((quint8)0x01);
    apsReq.setSrcEndpoint((quint8)0x01);
    apsReq.setClusterId(clusterID);
    apsReq.setProfileId(HA_PROFILE_ID);

    // prepare payload
    QDataStream stream(&apsReq.asdu(), QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    // generate and remember a new APS transaction sequence number
    m_readAttributeZCLSeq = (uint8_t)qrand();
    
    
    deCONZ::ZclFrame zclFrame;
    zclFrame.setSequenceNumber(m_readAttributeZCLSeq);
    zclFrame.setCommandId(cmdId); 

    DBG_Printf(DBG_INFO, "SEND ZCl COMMAND WITH THIS CLUSTER ID %d TO THIS ADDRESS %d and TM = %d and ONOFF = %d", clusterID, dstNode, TM_PROFILE_ID, ONOFF_PROFILE_ID);
    
    if (clusterID == TM_PROFILE_ID)
    {
	zclFrame.setFrameControl(deCONZ::ZclFCProfileCommand |
				 deCONZ::ZclFCDirectionClientToServer |
				 deCONZ::ZclFCDisableDefaultResponse);
    }
    if (clusterID == ONOFF_PROFILE_ID)
    {
	zclFrame.setFrameControl(deCONZ::ZclFCClusterCommand |
				 deCONZ::ZclFCDirectionClientToServer |
				 deCONZ::ZclFCDisableDefaultResponse);
    }
	
    zclFrame.writeToStream(stream);

    if (clusterID == TM_PROFILE_ID)
    {
	stream << (qint16)0x0000;       // Command read attribute
    }

    if (m_apsCtrl && (m_apsCtrl->apsdeDataRequest(apsReq) == deCONZ::Success))
    {
        // remember request
        m_apsReqQueue.push_back(apsReq);
        return true;
    }

    return false;
}

/*!
  Read the response of an ZCl RA request
 */
void BasicApsPlugin::handleZCLReadAttributeResponse(const deCONZ::ApsDataIndication &ind, deCONZ::ZclFrame &zclFrame)
{
    QString srcAddress = ind.srcAddress().toStringNwk();

    bool ok = false;

    quint16 srcAdd = srcAddress.toInt(&ok, 16);
    quint16 dstNode = (quint16)DST_NODE;

    qint16 temperature = -100;

    if ( ! ok )
    {
     	DBG_Printf(DBG_INFO, "Problem with the int conversion");
     	return; 
    }
    
    // if (srcAdd != dstNode) // We only want the temperature sensor responses 
    // {
    // 	// DBG_Printf(DBG_INFO, "srcAdd: %d - srcAddress : %s and DST_NODE : %d\n", srcAdd, srcAddress.toUtf8().constData(), dstNode);
    // 	return;
    // }
    
    QDataStream stream(zclFrame.payload());
    stream.setByteOrder(QDataStream::LittleEndian);

    bool isReadAttr = false;

    if (zclFrame.isProfileWideCommand() && zclFrame.commandId() == deCONZ::ZclReadAttributesResponseId)
    {
	isReadAttr = true;
    }

    if (isReadAttr)
    {
	DBG_Printf(DBG_INFO, "received ZCL RA response (id: %u) from %s\n", m_readAttributeZCLSeq, ind.srcAddress().toStringNwk().toUtf8().constData());
	
        while (!stream.atEnd())
        {
	    quint16 attrId;
	    quint8 attrTypeId;

	    stream >> attrId;
	    if (isReadAttr)
	    {
		quint8 status;
		stream >> status;  // Read Attribute Response status
		if (status != deCONZ::ZclSuccessStatus)
		{
		    continue;
		}
	    }
	    stream >> attrTypeId;

	    deCONZ::ZclAttribute attr(attrId, attrTypeId, QLatin1String(""), deCONZ::ZclRead, false);
	    
	    if (!attr.readFromStream(stream))
	    {
		continue;
	    }

	    if (attrId == 0x0000 && srcAdd == dstNode) // Local Temperature
	    {
		temperature = attr.numericValue().s16;
		DBG_Printf(DBG_INFO, "\tZCl RA temperature: %d\n", temperature);
	    }
        }
    }

    // Handle temperature
    if (temperature != -100) // We check if the var temperature is initialized 
    {

	// the temperature is good, not too cold not to hot
	if (temperature < TM_MAX && temperature > TM_MIN)
	{
	    return;
	}
	
	// We check the temperature and verify that the command has not been already sent
	// cmd is False by default -> No command
	// if cmd = true -> cmd on sent
	if (temperature < TM_MIN && !cmd)
	{
	    // We will send on command
	    if (sendZCLReadAttributeRequest((quint16)ACT_NODE, ONOFF_COMMAND_ON, ONOFF_PROFILE_ID))
            {
                DBG_Printf(DBG_INFO, "send ZCl RA request (id: %u)\n", m_readAttributeZCLSeq);
                setState(StateWaitSimpleDescriptorResponse);
                m_timer->start(WAIT_MATCH_DESCR_RESP_TIMEOUT);
		cmd = true; 
            }
	}
	// The temperature is good and the command is still on
	// Go send off command
	if (temperature > TM_MAX && cmd)
	{    
	    if (sendZCLReadAttributeRequest((quint16)ACT_NODE, ONOFF_COMMAND_OFF, ONOFF_PROFILE_ID))
            {
                DBG_Printf(DBG_INFO, "send ZCl RA request (id: %u)\n", m_readAttributeZCLSeq);
                setState(StateWaitSimpleDescriptorResponse);
                m_timer->start(WAIT_MATCH_DESCR_RESP_TIMEOUT);
		cmd = false; 
            }
	}
    }
}

/*! Sends a ZDP Match_Descr_req for On/Off cluster (ClusterID=0x0006).
    \return true if request was added to queue
 */
bool BasicApsPlugin::sendMatchDescriptorRequest()
{
    DBG_Assert(m_state == StateIdle);

    if (m_apsCtrl->networkState() != deCONZ::InNetwork)
    {
        return false;
    }

    deCONZ::ApsDataRequest apsReq;

    // set destination addressing
    apsReq.setDstAddressMode(deCONZ::ApsNwkAddress);
    apsReq.dstAddress().setNwk(deCONZ::BroadcastRxOnWhenIdle);
    apsReq.setDstEndpoint(ZDO_ENDPOINT);
    apsReq.setSrcEndpoint(ZDO_ENDPOINT);
    apsReq.setProfileId(ZDP_PROFILE_ID);
    apsReq.setClusterId(ZDP_MATCH_DESCRIPTOR_CLID);

    // prepare payload
    QDataStream stream(&apsReq.asdu(), QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    // generate and remember a new ZDP transaction sequence number
    m_matchDescrZdpSeq = (uint8_t)qrand();

    // write payload according to ZigBee specification (2.4.3.1.7 Match_Descr_req)
    // here we search for ZLL device which provides a OnOff server cluster
    // NOTE: explicit castings ensure correct size of the fields
    stream << m_matchDescrZdpSeq; // ZDP transaction sequence number
    stream << (quint16)deCONZ::BroadcastRxOnWhenIdle; // NWKAddrOfInterest
    stream << (quint16)ZLL_PROFILE_ID; // ProfileID
    stream << (quint8)0x01; // NumInClusters
    stream << (quint16)0x0006; // OnOff ClusterID
    stream << (quint8)0x00; // NumOutClusters

    if (m_apsCtrl && (m_apsCtrl->apsdeDataRequest(apsReq) == deCONZ::Success))
    {
        // remember request
        m_apsReqQueue.push_back(apsReq);
        return true;
    }

    return false;
}

/*! Sends a ZDP Simple_Descr_req for cluster (ClusterID=0x0004).
    \return true if request was added to queue
 */
bool BasicApsPlugin::sendSimpleDescriptorRequest()
{
    DBG_Assert(m_state == StateIdle);

    if (m_apsCtrl->networkState() != deCONZ::InNetwork)
    {
        return false;
    }

    deCONZ::ApsDataRequest apsReq;

    // set destination addressing
    apsReq.setDstAddressMode(deCONZ::ApsNwkAddress);
    apsReq.dstAddress().setNwk((quint16)DST_NODE);
    apsReq.setDstEndpoint(ZDO_ENDPOINT);
    apsReq.setSrcEndpoint(ZDO_ENDPOINT);
    apsReq.setProfileId(ZDP_PROFILE_ID);
    apsReq.setClusterId(ZDP_SIMPLE_DESCRIPTOR_CLID);

    // prepare payload
    QDataStream stream(&apsReq.asdu(), QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    // generate and remember a new ZDP transaction sequence number
    m_simpleDescrZdpSeq = (uint8_t)qrand();

    // write payload according to ZigBee specification (2.4.3.1.7 Simple_Descr_req)
    // NOTE: explicit castings ensure correct size of the fields
    stream << m_simpleDescrZdpSeq; // ZDP transaction sequence number
    //stream << (quint16)deCONZ::BroadcastRxOnWhenIdle; // NWKAddrOfInterest
    stream << (quint16)DST_NODE; // NWKAddrOfInterest
    // stream << (quint8)0x01; // NumInClusters
    stream << (quint16)0x0001; // Set the endpoint field in ZDP


    if (m_apsCtrl && (m_apsCtrl->apsdeDataRequest(apsReq) == deCONZ::Success))
    {
        // remember request
        m_apsReqQueue.push_back(apsReq);
        return true;
    }

    return false;
}

/*! Sets the state machine state.
    \param state the new state
 */
void BasicApsPlugin::setState(BasicApsPlugin::State state)
{
    if (m_state != state)
    {
        m_state = state;
    }
}

/*! deCONZ will ask this plugin which features are supported.
    \param feature - feature to be checked
    \return true if supported
 */
bool BasicApsPlugin::hasFeature(Features feature)
{
    switch (feature)
    {
    default:
        break;
    }

    return false;
}

/*! Main state machine event handler.
    \param event the event which occured
 */
void BasicApsPlugin::stateMachineEventHandler(BasicApsPlugin::Event event)
{
    if (m_state == StateIdle)
    {
        if (event == EventTimeout)
        {
            m_apsReqQueue.clear();

            // By default we send a read attribute command for Temperature measurement cluster ID
            // To the 0x7dff sensor node
            if (sendZCLReadAttributeRequest((quint16)DST_NODE, (quint8)0x00, TM_PROFILE_ID))
            {
                DBG_Printf(DBG_INFO, "send ZCl RA request (id: %u)\n", m_readAttributeZCLSeq);
                setState(StateWaitSimpleDescriptorResponse);
                m_timer->start(WAIT_MATCH_DESCR_RESP_TIMEOUT);
            }

	    if (sendSimpleDescriptorRequest())
            {
                DBG_Printf(DBG_INFO, "send simple descriptor request (id: %u)\n", m_simpleDescrZdpSeq);
                setState(StateWaitMatchDescriptorResponse);
                m_timer->start(WAIT_SIMPLE_DESCR_RESP_TIMEOUT);
            }

	    /*! Bluetooth program
	     */
	    btle_temperature = getTemp();
	    if (btle_temperature != -100)
	    {
		DBG_Printf(DBG_INFO, "Bluetooth command sent and recevied (T: %ld)\n", btle_temperature); 
                setState(StateBTLE);
                m_timer->start(WAIT_BTLE_TIMEOUT);
		m_btle = 1;
	    }

	    else
            {
                // try again later
                m_timer->start(IDLE_TIMEOUT);
            }
        }
    }
    else if (m_state == StateWaitMatchDescriptorResponse)
    {
        if (event == EventSendDone)
        {
            DBG_Printf(DBG_INFO, "send ZCL RA done (id: %u)\n", m_readAttributeZCLSeq);
        }
        else if (event == EventSendFailed)
        {
            DBG_Printf(DBG_INFO, "send ZCL RA request failed (id: %u)\n", m_readAttributeZCLSeq);
            // go back to idle state and wait some time
            setState(StateIdle);
            m_timer->start(IDLE_TIMEOUT);
        }
        else if (event == EventTimeout)
        {
            DBG_Printf(DBG_INFO, "stop wait for ZCL RA response (id: %u)\n", m_readAttributeZCLSeq);
            // go back to idle state and wait some time
            setState(StateIdle);
            m_timer->start(IDLE_TIMEOUT);
        }
    }
    else if (m_state == StateWaitSimpleDescriptorResponse)
    {
        if (event == EventSendDone)
        {
            DBG_Printf(DBG_INFO, "send simple descriptor request done (id: %u)\n", m_simpleDescrZdpSeq);
        }
        else if (event == EventSendFailed)
        {
            DBG_Printf(DBG_INFO, "send simple descriptor request failed (id: %u)\n", m_simpleDescrZdpSeq);
            // go back to idle state and wait some time
            setState(StateIdle);
            m_timer->start(IDLE_TIMEOUT);
        }
        else if (event == EventTimeout)
        {
            DBG_Printf(DBG_INFO, "stop wait for simple descriptor response (id: %u)\n", m_simpleDescrZdpSeq);
            // go back to idle state and wait some time
            setState(StateIdle);
            m_timer->start(IDLE_TIMEOUT);
        }
    }
    else if (m_state == StateBTLE)
    {
	// go back to idle state and wait some time
	setState(StateIdle);
	m_timer->start(IDLE_TIMEOUT);
    }
}

/*! Returns the name of this plugin.
 */
const char *BasicApsPlugin::name()
{
    return "Basic APS Plugin";
}

/*! Execute a python script mainly used to run bluetooth functions.
 */
long int BasicApsPlugin::getTemp()
{
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
    const char* module = "getTemp";


    Py_Initialize();
    pName = PyString_FromString("getTemp");
    pValue = PyString_FromString("DC:D0:17:9D:1D:5D");
    pArgs = PyTuple_New(1);
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    //DBG_Printf(DBG_INFO, "Cannot find function \"%s\"\n", PyString_AsString(pModule));  
    
    if (pModule != NULL)
    {
	pFunc = PyObject_GetAttrString(pModule, module);
	if (pFunc && PyCallable_Check(pFunc))
        {
	    if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
	    
	    PyTuple_SetItem(pArgs, 0, pValue);
            pValue = PyObject_CallObject(pFunc, pArgs);
	    Py_DECREF(pArgs);

	    if (pValue != NULL)
	    {
		printf("Result of call: %ld\n", PyInt_AsLong(pValue));
		return PyInt_AsLong(pValue);
	    }
	    else
	    {
		Py_DECREF(pFunc);
		Py_DECREF(pModule);
		PyErr_Print();
		fprintf(stderr,"Call failed\n");
		return 1;
	    }
	}
	else
	{
	    if (PyErr_Occurred())
		PyErr_Print();

	    fprintf(stderr, "Cannot find function \"%s\"\n", PyString_AsString(pFunc));
	}
	Py_XDECREF(pFunc);
	Py_DECREF(pModule);
    }
    else
    {
	PyErr_Print();
	fprintf(stderr, "Failed to load \"%s\"\n", PyString_AsString(pModule));
	return 1;
    }
    Py_Finalize();
    return 0;
}


/*!
  Handle the response received from the bluetooth program
 */
void BasicApsPlugin::handleBTLEResponse()
{
    int tm_max = TM_MAX / 100;
    int tm_min = TM_MIN / 100;
    // Handle temperature
    if (btle_temperature != -100) // We check if the var temperature is initialized 
    {
	
	// the temperature is good, not too cold not to hot
	if (btle_temperature < tm_max && btle_temperature > tm_min)
	{
	    return;
	}
	
	// We check the temperature and verify that the command has not been already sent
	// cmd is False by default -> No command
	// if cmd = true -> cmd on sent
	if (btle_temperature < tm_min && !cmd)
	{
	    // We will send on command
	    if (sendZCLReadAttributeRequest((quint16)ACT_NODE, ONOFF_COMMAND_ON, ONOFF_PROFILE_ID))
            {
                DBG_Printf(DBG_INFO, "send ZCl RA request (id: %u)\n", m_readAttributeZCLSeq);
                setState(StateWaitSimpleDescriptorResponse);
                m_timer->start(WAIT_MATCH_DESCR_RESP_TIMEOUT);
		cmd = true; 
            }
	}
	// The temperature is good and the command is still on
	// Go send off command
	if (btle_temperature > tm_max && cmd)
	{    
	    if (sendZCLReadAttributeRequest((quint16)ACT_NODE, ONOFF_COMMAND_OFF, ONOFF_PROFILE_ID))
            {
                DBG_Printf(DBG_INFO, "send ZCl RA request (id: %u)\n", m_readAttributeZCLSeq);
                setState(StateWaitSimpleDescriptorResponse);
                m_timer->start(WAIT_MATCH_DESCR_RESP_TIMEOUT);
		cmd = false; 
            }
	}
    }

    m_btle = 0;
}


#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(basic_aps_plugin, BasicApsPlugin)
#endif

