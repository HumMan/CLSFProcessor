#include "CLSFNodes.h"


void Save(pugi::xml_node to_xml,const char* name,vector<double>& v)
{
    pugi::xml_node v_node=to_xml.append_child(name);
    for(vector<double>::iterator i=v.begin();i!=v.end();i++)
        v_node.append_child("double").append_attribute("value").set_value(*i);
}

void Load(pugi::xml_node from_xml,const char* name,vector<double>& v)
{
    pugi::xml_node v_node=from_xml.child(name);
    for (pugi::xml_node_iterator it = v_node.begin(); it != v_node.end(); ++it)
        v.push_back(it->attribute("value").as_double());
}

void Save(pugi::xml_node to_xml,const char* name,string& v)
{
    to_xml.append_attribute(name).set_value(v.c_str());
}

void Load(pugi::xml_node from_xml,const char* name,string& v)
{
    v=from_xml.attribute(name).value();
}

void Save(pugi::xml_node to_xml,const char* name,Vector3d& v)
{
    pugi::xml_node t=to_xml.append_child(name);
    for(int i=0;i<3;i++)
    {
        char buff[10];
        sprintf(buff,"%i",i);
        t.append_attribute(buff)=v[i];
    }
}

void Load(pugi::xml_node from_xml,const char* name,Vector3d& v)
{
    pugi::xml_node t=from_xml.child(name);
    for(int i=0;i<3;i++)
    {
        char buff[10];
        sprintf(buff,"%i",i);
        v[i]=t.attribute(buff).as_double();
    }
}

void TMovement::Save(pugi::xml_node to_xml)
{
}

void TMovement::Load(pugi::xml_node from_xml)
{
}

void TProgram::Save(pugi::xml_node to_xml)
{
	pugi::xml_node tool_paths_node=to_xml.append_child("tool_paths");
	for(list<TToolPath>::iterator i=tool_paths.begin();i!=tool_paths.end();i++)
	{
		pugi::xml_node tool_path_node=tool_paths_node.append_child("tool_path");
		i->Save(tool_path_node);
	}
}

void TProgram::Load(pugi::xml_node from_xml)
{
	pugi::xml_node tool_paths_node=from_xml.child("tool_paths");
	for (pugi::xml_node_iterator it = tool_paths_node.begin(); it != tool_paths_node.end(); ++it)
	{
		tool_paths.push_back(TToolPath());
		tool_paths.back().Load(*it);
	}
}

void TToolPath::Save(pugi::xml_node to_xml)
{
        path_data.Save(to_xml.append_child("path_data"));
        tool_data.Save(to_xml.append_child("tool_data"));
        msys.Save(to_xml.append_child("msys"));
        pugi::xml_node movements_node=to_xml.append_child("movements");
        for(list<TMovement>::iterator i=movements.begin();i!=movements.end();i++)
        {
                pugi::xml_node movements_node=movements_node.append_child("movement");
                i->Save(movements_node);
        }
        if(has_spindl_rpm)
            to_xml.append_attribute("spindl_rpm")=spindl_rpm;
        if(has_tool_id)
            to_xml.append_attribute("tool_id")=tool_id;
        if(has_tool_adjust)
            to_xml.append_attribute("tool_adjust")=tool_adjust;
}

void TToolPath::Load(pugi::xml_node from_xml)
{
    path_data.Load(from_xml.child("path_data"));
    tool_data.Load(from_xml.child("tool_data"));
    msys.Load(from_xml.child("msys"));
    pugi::xml_node movements_node=from_xml.child("movements");
    for (pugi::xml_node_iterator it = movements_node.begin(); it != movements_node.end(); ++it)
    {
            movements.push_back(TMovement());
            movements.back().Load(*it);
    }
    if(!from_xml.attribute("spindl_rpm").empty())
    {
        has_spindl_rpm=true;
        spindl_rpm=from_xml.attribute("spindl_rpm").as_double();
    }else has_spindl_rpm=false;
    if(!from_xml.attribute("tool_id").empty())
    {
        has_tool_id=true;
        tool_id=from_xml.attribute("tool_id").as_int();
    }else has_tool_id=false;
    if(!from_xml.attribute("tool_adjust").empty())
    {
        has_tool_adjust=true;
        tool_adjust=from_xml.attribute("tool_adjust").as_int();
    }else has_tool_adjust=false;
}
