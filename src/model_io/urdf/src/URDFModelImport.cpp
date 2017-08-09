/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>
#include <iDynTree/ModelIO/URDFSolidShapesImport.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Transform.h>

#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/PrismaticJoint.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ModelTransformers.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>

#include <tinyxml.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <set>

#include "URDFParsingUtils.h"

namespace iDynTree
{

double URDFImportTol = 1e-9;

bool inertiaFromURDFXML(TiXmlElement * inertiaXml,
                        SpatialInertia & inertia)
{
    // Parse link_T_com transform
    // by default is the identity
    Transform link_T_com = Transform::Identity();

    TiXmlElement *o = inertiaXml->FirstChildElement("origin");
    if (o)
    {
        if (!transformFromURDFXML(o, link_T_com))
        return false;
    }

    TiXmlElement *mass_xml = inertiaXml->FirstChildElement("mass");
    if (!mass_xml)
    {
        reportError("","inertiaFromURDFXML","Inertial element must have a mass element");
        return false;
    }
    if (!mass_xml->Attribute("value"))
    {
        reportError("","inertiaFromURDFXML","Inertial: mass element must have value attribute");
        return false;
    }

    double mass;
    if( !stringToDoubleWithClassicLocale(mass_xml->Attribute("value"),mass) )
    {
        std::stringstream stm;
        stm << "Inertial: mass [" << mass_xml->Attribute("value")
            << "] is not a float";
        reportError("","inertiaFromURDFXML",stm.str().c_str());
        return false;
    }

    TiXmlElement *inertia_xml = inertiaXml->FirstChildElement("inertia");
    if (!inertia_xml)
    {
        reportError("","inertiaFromURDFXML","Inertial element must have inertia element");
        return false;
    }
    if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
            inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
            inertia_xml->Attribute("izz")))
    {
        reportError("","inertiaFromURDFXML","Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
        return false;
    }

    double ixx, ixy, ixz, iyy, iyz, izz;
    if( !stringToDoubleWithClassicLocale(inertia_xml->Attribute("ixx"),ixx)
        || !stringToDoubleWithClassicLocale(inertia_xml->Attribute("ixy"),ixy)
        || !stringToDoubleWithClassicLocale(inertia_xml->Attribute("ixz"),ixz)
        || !stringToDoubleWithClassicLocale(inertia_xml->Attribute("iyy"),iyy)
        || !stringToDoubleWithClassicLocale(inertia_xml->Attribute("iyz"),iyz)
        || !stringToDoubleWithClassicLocale(inertia_xml->Attribute("izz"),izz) )

    {
        std::stringstream stm;
        stm << "Inertial: one of the inertia elements is not a valid double:"
            << " ixx [" << inertia_xml->Attribute("ixx") << "]"
            << " ixy [" << inertia_xml->Attribute("ixy") << "]"
            << " ixz [" << inertia_xml->Attribute("ixz") << "]"
            << " iyy [" << inertia_xml->Attribute("iyy") << "]"
            << " iyz [" << inertia_xml->Attribute("iyz") << "]"
            << " izz [" << inertia_xml->Attribute("izz") << "]";
        reportError("","inertiaFromURDFXML",stm.str().c_str());
        return false;
    }

    // Create rotational inertia
    double rotInertiaWrtComData[3*3] = { ixx, ixy, ixz,
                                         ixy, iyy, iyz,
                                         ixz, iyz, izz};
    RotationalInertiaRaw rotInertiaWrtCom(rotInertiaWrtComData,3,3);

    // We need to rotate the inertia in the link reference frame
    // and use a special constructor-like function that takes in
    // to account that is expressed with respect to the com
    Position com_wrt_link = link_T_com.getPosition();
    Rotation link_R_com   = link_T_com.getRotation();
    inertia.fromRotationalInertiaWrtCenterOfMass(mass,com_wrt_link,link_R_com*rotInertiaWrtCom);

    return true;
}

bool linkFromURDFXML(TiXmlElement* linkXml,
                 Link & link,
                 std::string & linkName)
{
    const char *name_char = linkXml->Attribute("name");
    if (!name_char)
    {
        reportError("","linkFromURDFXML","No name given for a link in the model.");
        return false;
    }
    linkName = std::string(name_char);

    // Inertial (optional)
    TiXmlElement *i = linkXml->FirstChildElement("inertial");
    SpatialInertia inertia = SpatialInertia::Zero();
    if (i)
    {
        if (!inertiaFromURDFXML(i,inertia))
        {
            std::string errStr = "Could not parse inertial element for link " + linkName;
            reportError("","linkFromURDFXML",errStr.c_str());
            return false;
        }
    }
    link.setInertia(inertia);

    return true;
}

bool axisFromURDFXML(TiXmlElement* axisXml,
                     Axis         & axis)
{
    if (!axisXml || !(axisXml->Attribute("xyz")))
    {
        axis = Axis(Direction(1.0, 0.0, 0.0),Position(0.0,0.0,0.0));
    }
    else
    {
        const char* xyz_str = axisXml->Attribute("xyz");
        Vector3 xyz;
        if( vector3FromString(xyz_str,xyz) )
        {
            axis = Axis(Direction(xyz(0), xyz(1), xyz(2)),Position(0.0,0.0,0.0));
        }
        else
        {
            return false;
        }
    }

    return true;
}

bool posLimitsFromURDFXML(TiXmlElement* limXml,
                          double & posLimMin, double & posLimMax, std::string & jointName)
{
    if ( limXml->Attribute("lower") )
    {
        if( !stringToDoubleWithClassicLocale(limXml->Attribute("lower"),posLimMin) )
        {
            std::stringstream stm;
            stm << " [" << limXml->Attribute("lower")
                << "] is not a double, failing the parsing.";
            reportError("","posLimitsFromURDFXML",stm.str().c_str());
            return false;
        }
    }
    else
    {
        if(jointName == "revolute")
            reportWarning("","posLimitsFromURDFXML","lower tag not present in revolute joint, defaulting to 0.0 .");
        else if(jointName == "prismatic")
            reportWarning("","posLimitsFromURDFXML","lower tag not present in prismatic joint, defaulting to 0.0 .");
        posLimMin = 0.0;
    }

    if ( limXml->Attribute("upper") )
    {
        if( !stringToDoubleWithClassicLocale(limXml->Attribute("upper"),posLimMax) )
        {
            std::stringstream stm;
            stm << " [" << limXml->Attribute("upper")
                << "] is not a double, failing the parsing.";
            reportError("","posLimitsFromURDFXML",stm.str().c_str());
            return false;
        }
    }
    else
    {
        if(jointName == "revolute")
            reportWarning("","posLimitsFromURDFXML","upper tag not present in revolute joint, defaulting to 0.0 .");
        else if(jointName == "prismatic")
            reportWarning("","posLimitsFromURDFXML","upper tag not present in prismatic joint, defaulting to 0.0 .");
        posLimMax = 0.0;
    }

    return true;
}


/**
 * This function allocated dinamically the joint.
 * The joint should be then deleted by the parser
 * after it has been added to the model.
 *
 */
bool jointFromURDFXML(const Model & model,
                      TiXmlElement* jointXml,
                      IJointPtr & p_joint,
                      std::string & jointName,
                      std::string & parentLinkName,
                      std::string & childLinkName,
                      std::string & jointType)
{
    // reset p_joint
    p_joint = 0;
    // Get Joint Name
    const char *name = jointXml->Attribute("name");
    if (!name)
    {
        reportError("","jointFromURDFXML","unnamed joint found");
        return false;
    }
    jointName = name;

    // Get transform from Parent Link to Joint Frame
    Transform parent_T_joint;
    TiXmlElement *origin_xml = jointXml->FirstChildElement("origin");
    if (!origin_xml)
    {
        parent_T_joint = Transform::Identity();
    }
    else
    {
        if (!transformFromURDFXML(origin_xml,parent_T_joint))
        {
            std::string errStr = "Malformed parent origin element for joint " + jointName;
            reportError("","jointFromURDFXML",errStr.c_str());
            return false;
        }
    }

    // Get Parent Link
    TiXmlElement *parent_xml = jointXml->FirstChildElement("parent");
    if (parent_xml)
    {
        const char *pname = parent_xml->Attribute("link");
        if (!pname)
        {
            std::string errStr = "No parent specified for joint " + jointName;
            reportError("","jointFromURDFXML",errStr.c_str());
            return false;
        }
        else
        {
            parentLinkName = std::string(pname);
        }
    }

    // Get Child Link
    TiXmlElement *child_xml = jointXml->FirstChildElement("child");
    if (child_xml)
    {
        const char *pname = child_xml->Attribute("link");
        if (!pname)
        {
            std::string errStr = "No child specified for joint " + jointName;
            reportError("","jointFromURDFXML",errStr.c_str());
            return false;
        }
        else
        {
            childLinkName = std::string(pname);
        }
    }

    assert(parentLinkName != childLinkName);

    // Get Joint type
    const char* type_char = jointXml->Attribute("type");
    if (!type_char)
    {
        std::string errStr = "No child specified for joint " + jointName;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    std::string type_str = type_char;
    jointType = type_str;
    if (type_str == "planar" ||
        type_str == "floating")
    {
        std::string errStr = "Joint " + jointName + " has type " + type_str + " that is not currently supported by iDynTree";
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }
    else if (type_str == "fixed" ||
             type_str == "revolute"  ||
             type_str == "continuous" ||
             type_str == "prismatic")
    {
        // perfect, type supported by iDynTree, parsing happening later
    }
    else
    {
        std::string errStr = "Joint " + jointName + " has unknown type " + type_str;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    // Get indices in model for the links involved in the joint
    LinkIndex parentLinkIndex = model.getLinkIndex(parentLinkName);

    if( parentLinkIndex == LINK_INVALID_INDEX )
    {
        std::string errStr = "Joint " + jointName + " has unknown parent " + parentLinkName;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    LinkIndex childLinkIndex  = model.getLinkIndex(childLinkName);

    if( childLinkIndex == LINK_INVALID_INDEX )
    {
        std::string errStr = "Joint " + jointName + " has unknown child " + childLinkName;
        reportError("","jointFromURDFXML",errStr.c_str());
        return false;
    }

    assert(parentLinkIndex != childLinkIndex);

     // Get Joint Axis
    Axis axis_wrt_childLink;
    if (type_str != "fixed" && type_str != "floating")
    {
        TiXmlElement *axis_xml = jointXml->FirstChildElement("axis");
        axisFromURDFXML(axis_xml,axis_wrt_childLink);
    }

    // Actually allocate a joint
    if( type_str == "fixed" )
    {
        p_joint = new FixedJoint(parentLinkIndex,childLinkIndex,parent_T_joint);
    }
    else if ( type_str == "revolute"
           || type_str == "continuous" )
    {
        RevoluteJoint * rev_joint = new RevoluteJoint();
        rev_joint->setAttachedLinks(parentLinkIndex,childLinkIndex);
        rev_joint->setRestTransform(parent_T_joint);
        rev_joint->setAxis(axis_wrt_childLink, childLinkIndex);
        p_joint = static_cast<IJoint*>(rev_joint);
    }
    else if (type_str == "prismatic")
    {
        PrismaticJoint * prism_joint = new PrismaticJoint();
        prism_joint->setAttachedLinks(parentLinkIndex,childLinkIndex);
        prism_joint->setRestTransform(parent_T_joint);
        prism_joint->setAxis(axis_wrt_childLink, childLinkIndex);
        p_joint = static_cast<IJoint*>(prism_joint);
    }

    assert(p_joint != 0);

    // Get Joint limits
    TiXmlElement *limit_xml = jointXml->FirstChildElement("limit");
    if (limit_xml)
    {
        double posLimMin, posLimMax;
        if( !posLimitsFromURDFXML(limit_xml,posLimMin,posLimMax,jointName) )
        {
            std::string errStr = "Parsing limits failed for joint " + jointName + ", URDF parsing failed.";
            reportError("", "jointFromURDFXML", errStr.c_str());
            delete p_joint;
            p_joint = 0;
            return false;
        }

        if( p_joint->getNrOfDOFs() == 0 )
        {
            std::string warnStr = "Limits are presented in joint " + jointName + ", that has zero DOFs.";
            reportWarning("", "jointFromURDFXML", warnStr.c_str());
        }

        p_joint->enablePosLimits(true);
        p_joint->setPosLimits(0,posLimMin,posLimMax);
    }
    else if (type_str == "revolute")
    {
        std::string errStr = "Joint " + jointName + " has type revolute but the limit tag is missing.";
        reportWarning("","jointFromURDFXML",errStr.c_str());
    }
    else if (type_str == "prismatic")
    {
        std::string errStr = "Joint " + jointName + " has type prismatic but the limit tag is missing.";
        reportWarning("","jointFromURDFXML",errStr.c_str());
    }

    return true;
}


bool modelFromURDF(const std::string & urdf_filename,
                         iDynTree::Model & output,
                         const URDFParserOptions options)
{
    std::ifstream ifs(urdf_filename.c_str());

    if( !ifs.is_open() )
    {
        std::cerr << "[ERROR] iDynTree::modelFromURDF : error opening file "
                  << urdf_filename << std::endl;
        return false;
    }

    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                            (std::istreambuf_iterator<char>()    ) );

    // We store the original filename, this is useful to find external meshes
    URDFParserOptions modifiedOptions = options;
    modifiedOptions.originalFilename = urdf_filename;

    return modelFromURDFString(xml_string,output,modifiedOptions);
}

/**
 * Check if a given transform is the identity, up to the transform.
 */
bool isIdentity(const Transform & trans)
{
    Rotation rot = trans.getRotation();
    Position pos = trans.getPosition();

    for(int row=0; row < 3; row++ )
    {
        for(int col=0; col < 3; col++ )
        {
            if( row == col )
            {
                if( fabs(rot(row,col)-1) > URDFImportTol )
                {
                    return false;
                }
            }
            else
            {
                if( fabs(rot(row,col)) > URDFImportTol )
                {
                    return false;
                }
            }
        }
    }

    for(int row=0; row < 3; row++ )
    {
        if( fabs(pos(row)) > URDFImportTol )
        {
            return false;
        }
    }

    return true;
}

/**
 * Helper function of modelFromURDFString , used to cleanup the
 * fixed joints temporary vector in case of error.
 *
 */
void cleanupFixedJoints(std::vector<IJointPtr> & fixedJoints)
{
    for(size_t i=0; i < fixedJoints.size(); i++)
    {
        if( fixedJoints[i] )
        {
            delete fixedJoints[i];
            fixedJoints[i] = 0;
        }
    }
}


/**
 * Helper function to load all the sensor frames from
 * as additional frames in the model, so they can be used
 * in all the methods that are frame-specific such as Jacobian computations.
 */
bool addSensorFramesAsAdditionalFramesToModel(Model & model,
                                              const SensorsList & sensors)
{
    bool ret = true;

    // First, we cycle on all the sensor that are attached to a link, because their frame is easy to add
    // TODO : not super happy about this cycle, but is doing is work well
    for(SensorType type=SIX_AXIS_FORCE_TORQUE; type < NR_OF_SENSOR_TYPES; type = (SensorType)(type+1))
    {
        // TODO : link sensor are extremly widespared and they have all approximatly the same API,
        //        so we need a better way to iterate over them
        if( isLinkSensor(type) )
        {
            for(size_t sensIdx = 0; sensIdx < sensors.getNrOfSensors(type); sensIdx++)
            {
                LinkSensor * linkSensor = dynamic_cast<LinkSensor*>(sensors.getSensor(type,sensIdx));

                LinkIndex   linkToWhichTheSensorIsAttached = linkSensor->getParentLinkIndex();
                std::string linkToWhichTheSensorIsAttachedName = model.getLinkName(linkToWhichTheSensorIsAttached);

                if( model.isFrameNameUsed(linkSensor->getName()) )
                {
                    std::string err = "addSensorFramesAsAdditionalFrames is specified as an option, but it is impossible to add the frame of sensor " + linkSensor->getName() + " as there is already a frame with that name";
                    reportWarning("","addSensorFramesAsAdditionalFramesToModel",err.c_str());
                }
                else
                {
                    // std::cerr << "Adding sensor " << linkSensor->getName() << " to link " << linkToWhichTheSensorIsAttachedName << " as additional frame"<< std::endl;
                    bool ok = model.addAdditionalFrameToLink(linkToWhichTheSensorIsAttachedName,linkSensor->getName(),linkSensor->getLinkSensorTransform());

                    if( !ok )
                    {
                        std::string err = "addSensorFramesAsAdditionalFrames is specified as an option, but it is impossible to add the frame of sensor " + linkSensor->getName() + " for unknown reasons";
                        reportError("","addSensorFramesAsAdditionalFramesToModel",err.c_str());
                        ret = false;
                    }
                }
            }
        }

        // Explictly address the case of F/T sensors
        if( type == SIX_AXIS_FORCE_TORQUE )
        {
            // We add the sensor frame as an additional frame of the **child** link
            // (as tipically for URDF sensors the child link frame is coincident with the F/T sensor frame
            for(size_t sensIdx = 0; sensIdx < sensors.getNrOfSensors(type); sensIdx++)
            {
                SixAxisForceTorqueSensor * ftSensor = dynamic_cast<SixAxisForceTorqueSensor*>(sensors.getSensor(type,sensIdx));

                std::string linkToWhichTheSensorIsAttachedName = ftSensor->getSecondLinkName();

                if( model.isFrameNameUsed(ftSensor->getName()) )
                {
                    std::string err = "addSensorFramesAsAdditionalFrames is specified as an option, but it is impossible to add the frame of sensor " + ftSensor->getName() + " as there is already a frame with that name";
                    reportWarning("","addSensorFramesAsAdditionalFramesToModel",err.c_str());
                }
                else
                {
                    Transform link_H_sensor;
                    bool ok = ftSensor->getLinkSensorTransform(ftSensor->getSecondLinkIndex(),link_H_sensor);
                    ok = ok && model.addAdditionalFrameToLink(linkToWhichTheSensorIsAttachedName,ftSensor->getName(),link_H_sensor);

                    if( !ok )
                    {
                        std::string err = "addSensorFramesAsAdditionalFrames is specified as an option, but it is impossible to add the frame of sensor " + ftSensor->getName() + " for unknown reasons";
                        reportError("","addSensorFramesAsAdditionalFramesToModel",err.c_str());
                        ret = false;
                    }
                }
            }
        }
    }

    return ret;
}

bool modelFromURDFString(const std::string& urdf_string,
                               iDynTree::Model& model,
                               const URDFParserOptions options)
{
    bool ok = true;

    // clear the input model
    Model rawModel = Model();

    TiXmlDocument urdfXml;
    if (urdfXml.Parse(urdf_string.c_str()) == 0) {
        reportError("", "modelFromURDFString", "Invalid XML");
        return false;
    }

    TiXmlElement* robotXml = urdfXml.FirstChildElement("robot");

    // parse all links
    for (TiXmlElement* link_xml = robotXml->FirstChildElement("link");
         link_xml; link_xml = link_xml->NextSiblingElement("link") )
    {
        Link link;
        std::string linkName;
        ok = ok && linkFromURDFXML(link_xml,link,linkName);
        if( !ok )
        {
            model = Model();
            return false;
        }

        LinkIndex newLinkIndex = rawModel.addLink(linkName,link);

        if( newLinkIndex == LINK_INVALID_INDEX )
        {
            model = Model();
            return false;
        }
    }

    // parse all joints, saving a set of parents and childs
    std::set<std::string> parents;
    std::set<std::string> childs;
    // we parse the fixed joints separatly, so we can add them
    // all after the joints with a non-zero dofs . This is necessary
    // for backward compatibility with KDL-based software
    std::vector<IJointPtr> fixedJoints;
    std::vector<std::string> fixedJointNames;
    for (TiXmlElement* joint_xml = robotXml->FirstChildElement("joint");
         joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
    {
        IJointPtr joint=0;
        std::string jointName;
        std::string parentLinkName;
        std::string childLinkName;
        std::string jointType;

        ok = ok && jointFromURDFXML(rawModel,joint_xml,joint,jointName,parentLinkName,childLinkName,jointType);

        // save parent and child in a set
        parents.insert(parentLinkName);
        childs.insert(childLinkName);

        if( !ok )
        {
            cleanupFixedJoints(fixedJoints);
            delete joint;
            model = Model();
            return false;
        }

        assert(joint->getFirstAttachedLink() != joint->getSecondAttachedLink());

        // All joints but the fixed one are immediatly added to the model
        if( jointType != "fixed" )
        {
            JointIndex newJointIndex = rawModel.addJoint(jointName,joint);

            delete joint;

            if( newJointIndex == JOINT_INVALID_INDEX )
            {
                cleanupFixedJoints(fixedJoints);
                model = Model();
                return false;
            }
        }
        else
        {
            // fixed joints are stored separatly and added after the non-zero dofs joints
            fixedJoints.push_back(joint);
            fixedJointNames.push_back(jointName);
        }

    }

    // Adding all the fixed joint in the end
    for(size_t i=0; i < fixedJoints.size(); i++)
    {
        assert( fixedJoints.size() == fixedJointNames.size() );
        JointIndex newJointIndex = rawModel.addJoint(fixedJointNames[i],fixedJoints[i]);

        delete fixedJoints[i];
        fixedJoints[i] = 0;

        // If we had a problem adding the fixed joint
        // we cleanup the loading and we exit
        if( newJointIndex == JOINT_INVALID_INDEX )
        {
            cleanupFixedJoints(fixedJoints);
            model = Model();
            return false;
        }
    }

    // Get root
    std::vector<std::string> rootCandidates;
    for(unsigned int lnk=0; lnk < rawModel.getNrOfLinks(); lnk++ )
    {
        std::string linkName = rawModel.getLinkName(lnk);

        if( childs.find(linkName) == childs.end() )
        {
            rootCandidates.push_back(linkName);
        }
    }

    if( rootCandidates.size() == 0 )
    {
        reportError("","modelFromURDFString","No root link found in URDF string");
        model = Model();
        return false;
    }

    if( rootCandidates.size() >= 2 )
    {
        std::stringstream ss;
        ss << "Multiple (" << rootCandidates.size() << ") root links: (";
        for(size_t root=0; root < rootCandidates.size(); root++)
        {
            ss << rootCandidates[root];

            if( root != rootCandidates.size()-1 )
            {
                ss << ", ";
            }
        }
        ss << ") found in URDF string";

        std::cerr << "Multiple roots!!!" << std::endl;
        reportError("","modelFromURDFString",ss.str().c_str());
        model = Model();
        return false;
    }

    // set the default root in the model
    rawModel.setDefaultBaseLink(rawModel.getLinkIndex(rootCandidates[0]));

    // Remove fake links and add them as frames
    ok = ok && removeFakeLinks(rawModel,model);

    if( options.addSensorFramesAsAdditionalFrames )
    {
        // Add sensor frames as additional frames in the model

        // We must first parse the sensor, as at the moment the two parsers are separated
        iDynTree::SensorsList sensors;
        sensorsFromURDFString(urdf_string,model,sensors);

        addSensorFramesAsAdditionalFramesToModel(model,sensors);
    }

    // Parse visual and collision geometries
    ok = ok && solidShapesFromURDFString(urdf_string,options.originalFilename,model,"visual",model.visualSolidShapes());
    ok = ok && solidShapesFromURDFString(urdf_string,options.originalFilename,model,"collision",model.collisionSolidShapes());

    return ok;
}

}
