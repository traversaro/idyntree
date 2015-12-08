/*
 * Copyright (C) 2015 RobotCub Consortium
 * Author: Nuno Guedelha
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include "stdio.h"

#include <yarp/math/Math.h>
#include "iCub/skinDynLib/dynContact.h"
#include <iCub/ctrl/math.h>
#include "iCub/iDynTree/iDynContact.h"

using namespace std;
using namespace iDynTree;
//using namespace yarp::os;
//using namespace yarp::sig;
//using namespace yarp::math;
//using namespace iCub::ctrl;
//using namespace iCub::skinDynLib;

//~~~~~~~~~~~~~~~~~~~~~~
//   DYN CONTACT
//~~~~~~~~~~~~~~~~~~~~~~
iDynContact::iDynContact():
contactId(-1),
linkIndex(-1),
frameIndex(-1)
{
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynContact::iDynContact(const unsigned long _contactId, const unsigned int _linkIndex,
                         unsigned int _frameIndex, const Position & _CoP,
                         const Force & _Fdir, const Moment & _Mu):
contactId(_contactId),
linkIndex(_linkIndex),
frameIndex(_frameIndex)
{
//    if (_Fdir==NULL)
//    {
//        Fdir = Position::Zero();
//        fDirKnown = false;
//    }
//    else
//    {
//        Fdir = _Fdir;
//        fDirKnown = true;
//    }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if 0
void iDynContact::init(const BodyPart &_bodyPart, unsigned int _linkNumber, const Vector &_CoP, const Vector &_Mu, const Vector &_Fdir){
    contactId = ID++;
    setBodyPart(_bodyPart);
    setLinkNumber(_linkNumber);
    setCoP(_CoP);
    Mu.resize(3, 0.0);
    Fdir.resize(3, 0.0);
    F.resize(3, 0.0);
    Fmodule = 0.0;

    if(_Mu.size()==0)
        muKnown = false;
    else
        fixMoment(_Mu);

    if(_Fdir.size()==0)
        fDirKnown = false;
    else
        fixForceDirection(_Fdir);
}
//~~~~~~~~~~~~~~~~~~~~~~
//   GET methods
//~~~~~~~~~~~~~~~~~~~~~~
Vector iDynContact::getForceMoment() const{ return cat(F, Mu); }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& iDynContact::getForce() const{ return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& iDynContact::getForceDirection() const{ return Fdir;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iDynContact::getForceModule() const{ return Fmodule;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& iDynContact::getMoment() const{ return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& iDynContact::getCoP() const{ return CoP;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iDynContact::getLinkNumber() const{ return linkNumber;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BodyPart iDynContact::getBodyPart() const{ return bodyPart;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynContact::getBodyPartName() const{ return BodyPart_s[bodyPart];}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned long iDynContact::getId() const{ return contactId;}

//~~~~~~~~~~~~~~~~~~~~~~
//   IS methods
//~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::isMomentKnown() const{ return muKnown;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::isForceDirectionKnown() const{ return fDirKnown;}
//~~~~~~~~~~~~~~~~~~~~~~
//   SET methods
//~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForce(const Vector &_F){
    if(!checkVectorDim(_F, 3, "force"))
        return false;
    F = _F;
    Fmodule = norm(_F);
    if(Fmodule!=0.0)
        Fdir = _F / Fmodule;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForceModule(double _Fmodule){
    if(_Fmodule<0){
        if(verbose)
            fprintf(stderr, "Error in iDynContact: negative force module, %f\n", _Fmodule);
        return false;
    }
    Fmodule = _Fmodule;
    F=Fmodule*Fdir;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForceDirection(const Vector &_Fdir){
    if(!checkVectorDim(_Fdir, 3, "force direction"))
        return false;
    double FdirNorm = norm(_Fdir);
    if(FdirNorm != 0.0)
        Fdir = _Fdir / FdirNorm;
    F=Fmodule*Fdir;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setMoment(const Vector &_Mu){
    if(!checkVectorDim(_Mu, 3, "moment"))
        return false;
    Mu = _Mu;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForceMoment(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu){
    return setForce(_F) && setMoment(_Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setForceMoment(const yarp::sig::Vector &_FMu){
    if(!checkVectorDim(_FMu, 6, "force moment"))
        return false;
    bool res = setForce(_FMu.subVector(0,2));
    return res && setMoment(_FMu.subVector(3,5));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::setCoP(const Vector &_CoP){
    if(!checkVectorDim(_CoP, 3, "Center of pressure"))
        return false;
    CoP = _CoP;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::setLinkNumber(unsigned int _linkNum){
    linkNumber = _linkNum;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::setBodyPart(BodyPart _bodyPart){
    bodyPart = _bodyPart;
}
//~~~~~~~~~~~~~~~~~~~~~~
//   FIX/UNFIX methods
//~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::fixForceDirection(const Vector &_Fdir){
    if(setForceDirection(_Fdir)){
        fDirKnown = true;
        return true;
    }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::fixMoment(){
    return fixMoment(zeros(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::fixMoment(const Vector &_Mu){
    if(setMoment(_Mu)){
        muKnown = true;
        return true;
    }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::unfixForceDirection(){ fDirKnown=false;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::unfixMoment(){ muKnown=false;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~
//   SERIALIZATION methods
//~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::write(ConnectionWriter& connection){
    // represent a iDynContact as a list of 4 elements that are:
    // - a list of 3 int, i.e. contactId, bodyPart, linkNumber
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment

    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(4);
    // list of 3 int, i.e. contactId, bodyPart, linkNumber
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_INT);
    connection.appendInt(3);
    connection.appendInt(contactId);
    connection.appendInt(bodyPart);    // left_arm, right_arm, ...
    connection.appendInt(linkNumber);
    // list of 3 double, i.e. the CoP
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(CoP[i]);
    // list of 3 double, i.e. the force
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(F[i]);
    // list of 3 double, i.e. the moment
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(Mu[i]);

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::read(ConnectionReader& connection){
    // auto-convert text mode interaction
    connection.convertTextMode();

    // represent a iDynContact as a list of 4 elements that are:
    // - a list of 3 int, i.e. contactId, bodyPart, linkNumber
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    if(connection.expectInt()!= BOTTLE_TAG_LIST || connection.expectInt()!=4)
        return false;
    // - a list of 3 int, i.e. contactId, bodyPart, linkNumber
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT || connection.expectInt()!=3)
        return false;
    contactId   = connection.expectInt();
    bodyPart    = (BodyPart)connection.expectInt();
    linkNumber  = connection.expectInt();
    // - a list of 3 double, i.e. the CoP
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) CoP[i] = connection.expectDouble();
    // - a list of 3 double, i.e. the force
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) F[i] = connection.expectDouble();
    setForce(F);
    // - a list of 3 double, i.e. the moment
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) Mu[i] = connection.expectDouble();

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynContact::toString(int precision) const{
    stringstream res;
    res<< "Contact id: "<< contactId<< ", Body part: "<< BodyPart_s[bodyPart]<< ", link: "<< linkNumber<< ", CoP: "<<
    CoP.toString(precision)<< ", F: "<< F.toString(precision)<< ", M: "<< Mu.toString(precision);
    return res.str();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iDynContact::setVerbose(unsigned int verb){
    verbose = verb;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iDynContact::checkVectorDim(const Vector &v, unsigned int dim, const string &descr){
    if(v.length() != dim){
        if(verbose)
            fprintf(stderr, "Error in iDynContact: unexpected dimension of vector %s, %d\n", descr.c_str(), (int)v.length());
        return false;
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#endif
