/*
 * Copyright (C) 2013 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef BASIC_APS_PLUGIN_H
#define BASIC_APS_PLUGIN_H

#include <QObject>
#include <list>
#include <deconz.h>

#if DECONZ_LIB_VERSION < 0x010100
  #error "The basic aps plugin requires at least deCONZ library version 1.1.0."
#endif

/*! \class BasicApsPlugin

    Plugin to demonstrate the library support for sending and receiving APS frames via
    APSDE-DATA.request, APSDE-DATA.confirm and APSDE-DATA.indication primitives.

    The plugin executes a state machine which does the following:

    1) Enter idle state and start timer
    2) On timout send broadcast request to search for lights
    3) Enter Wait state, start timer and wait for responses to print them
    4) On timeout go back to 1)

 */
class BasicApsPlugin : public QObject,
                       public deCONZ::NodeInterface
{
    Q_OBJECT
    Q_INTERFACES(deCONZ::NodeInterface)
#if QT_VERSION >= 0x050000
    Q_PLUGIN_METADATA(IID "org.dresden-elektronik.BasicApsPlugin")
#endif



public:
    //!< State machine states
    enum State
    {
        StateIdle,
        StateWaitMatchDescriptorResponse,
	StateWaitSimpleDescriptorResponse,
	StateBTLE
    };

    //!< State machine events
    enum Event
    {
        EventSendDone,
        EventSendFailed,
        EventTimeout
    };

    explicit BasicApsPlugin(QObject *parent = 0);
    ~BasicApsPlugin();
    // node interface
    const char *name();
    bool hasFeature(Features feature);

public Q_SLOTS:
    void stateMachineEventHandler(Event event);
    void apsdeDataIndication(const deCONZ::ApsDataIndication &ind);
    void apsdeDataConfirm(const deCONZ::ApsDataConfirm &conf);
    void handleZCLReadAttributeResponse(const deCONZ::ApsDataIndication &ind, deCONZ::ZclFrame &zclFrame);
    void handleMatchDescriptorResponse(const deCONZ::ApsDataIndication &ind);
    void handleSimpleDescriptorResponse(const deCONZ::ApsDataIndication &ind);
    void handleBTLEResponse();
    void timerFired();
    bool sendZCLReadAttributeRequest(quint16 node, quint8 cmdId, quint16 clusterId);
    bool sendMatchDescriptorRequest();
    bool sendSimpleDescriptorRequest();
    void setState(State state);
    long int getTemp();

private:
    State m_state; //!< current state machine state
    QTimer *m_timer; //!< common timer
    quint8 m_btle; //!< BTLE id that confirm the response
    long int btle_temperature; //!< Temperature returned by the btle program
    quint8 m_matchDescrZdpSeq; //!< ZDP transaction sequence number of Match_Descr_req
    quint8 m_simpleDescrZdpSeq; //!< ZDP transaction sequence number of Simple_Descr_req
    quint8 m_readAttributeZCLSeq;
    bool cmd;
    std::list<deCONZ::ApsDataRequest> m_apsReqQueue; //!< queue of active APS requests
    deCONZ::ApsController *m_apsCtrl; //!< pointer to ApsController instance
};

#endif // BASIC_APS_PLUGIN_H
