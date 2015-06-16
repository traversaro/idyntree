/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "RotationSemantics.h"
#include "PositionSemantics.h"
#include "Utils.h"
#include <iostream>
#include <sstream>

namespace iDynTree
{
    RotationSemantics::RotationSemantics(): orientationFrame(UNKNOWN),
                                            refOrientationFrame(UNKNOWN),
                                            coordinateFrame(UNKNOWN)
    {

    }

    RotationSemantics::RotationSemantics(int _orientationFrame, int _refOrientationFrame): orientationFrame(_orientationFrame),
                                                                                           refOrientationFrame(_refOrientationFrame),
                                                                                           coordinateFrame(_refOrientationFrame)
    {
    }


    RotationSemantics::RotationSemantics(const RotationSemantics& other)
    {
        this->orientationFrame = other.orientationFrame;
        this->refOrientationFrame = other.refOrientationFrame;
        this->coordinateFrame = other.coordinateFrame;
    }

    RotationSemantics::~RotationSemantics()
    {

    }

    int RotationSemantics::getOrientationFrame() const
    {
        return this->orientationFrame;
    }

    int RotationSemantics::getReferenceOrientationFrame() const
    {
        return this->refOrientationFrame;
    }

    int RotationSemantics::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }
    
    void RotationSemantics::setOrientationFrame(int _orientationFrame)
    {
        this->orientationFrame = _orientationFrame;
    }

    void RotationSemantics::setReferenceOrientationFrame(int _refOrientationFrame)
    {
        this->refOrientationFrame = _refOrientationFrame;
        this->coordinateFrame = _refOrientationFrame;
    }
    
    bool RotationSemantics::check_changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        // check semantics
        if( !checkEqualOrUnknown(this->orientationFrame,newOrientFrame.getReferenceOrientationFrame()) )
        {
            std::cerr << "[ERROR] RotationSemantics::changeOrientFrame : the orientationFrame of this object does not match the referenceOrientationFrame of the newOrientFrame\n";
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        // check semantics
        if( !checkEqualOrUnknown(newRefOrientFrame.getOrientationFrame(),this->refOrientationFrame) )
        {
            std::cerr << "[ERROR] RotationSemantics::changeRefOrientFrame : the refOrientationFrame of this object does not match the orientationFrame of the newRefOrientFrame\n";
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_compose(const RotationSemantics& op1, const RotationSemantics& op2)
    {
        // check semantics
        if( !checkEqualOrUnknown(op1.getOrientationFrame(),op2.getReferenceOrientationFrame()) )
        {
            std::cerr << "[ERROR] RotationSemantics::compose : the orientationFrame of the first operand does not match the referenceOrientationFrame of the second operand\n";
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_inverse2(const RotationSemantics& op)
    {
        return true;
    }

    bool RotationSemantics::check_transform(const RotationSemantics& op1, const PositionSemantics& op2)
    {
        // check semantics
        if( !checkEqualOrUnknown(op1.getOrientationFrame(),op2.getCoordinateFrame() ) )
        {
            std::cerr << "[ERROR] RotationSemantics::apply : the orientationFrame of the Rotation does not match the coordinateFrame of the position\n";
            return false;
        }

        return true;
    }


    bool RotationSemantics::changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        // check semantics
        bool status = RotationSemantics::check_changeOrientFrame(newOrientFrame);
        
        // set new semantics
        this->orientationFrame = newOrientFrame.orientationFrame;

        return status;
    }

    bool RotationSemantics::changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        // check semantics
        bool status = RotationSemantics::check_changeRefOrientFrame(newRefOrientFrame);
        
        // set new semantics
        this->refOrientationFrame = newRefOrientFrame.refOrientationFrame;
        
        return status;
    }

    bool RotationSemantics::compose(const RotationSemantics& op1, const RotationSemantics& op2, RotationSemantics& result)
    {
        // check semantics
        bool status = RotationSemantics::check_compose(<#const iDynTree::RotationSemantics &op1#>, <#const iDynTree::RotationSemantics &op2#>);
        
        // set new semantics
        result.refOrientationFrame = op1.getReferenceOrientationFrame();
        result.orientationFrame    = op2.getOrientationFrame();
    }

    bool RotationSemantics::compose(const RotationSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        result.setCoordinateFrame(op1.getReferenceOrientationFrame());
        result.setPoint(op2.getPoint());
        result.setReferencePoint(op2.getReferencePoint());
    }
    
    bool RotationSemantics::inverse2(const RotationSemantics& op, RotationSemantics& result)
    {
        result.refOrientationFrame = op.getOrientationFrame();
        result.orientationFrame    = op.getReferenceOrientationFrame();
    }

    PositionSemantics RotationSemantics::transform(const RotationSemantics& op1, const PositionSemantics& op2)
    {
        PositionSemantics result;

        RotationSemantics::transform(op1,op2,result);

        return result;
    }

    std::string RotationSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " orientationFrame " << this->getOrientationFrame()
           << " referenceOrientationFrame " << this->getReferenceOrientationFrame();

        return ss.str();
    }

    std::string RotationSemantics::reservedToString() const
    {
        return this->toString();
    }

}