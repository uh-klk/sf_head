//headMovement_Parcer: Reads head movement from a XML file, interpretes and returns
// a sequence of tones at a time taking into account of the sequence's
// timing
// Todo: - taking user preferences into account (preference from topic to compile the tune)
//       - better name for action

#ifndef HEADMOVEMENT_HPP_INCLUDED
#define HEADMOVEMENT_HPP_INCLUDED

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <cstring>
#include <unistd.h>

#include <time.h>

#include "rapidxml/rapidxml.hpp"

using namespace rapidxml;
using namespace std;

struct Action {
    int goal;
    int param;
    int modifier; //to indicate if this joint state should be modify based on user's preferences
};

struct SequenceOfActions { //Sequence of Actions
    int numberOfActions;
    vector<Action> action;
};

struct Behaviour { //XML structure
    string id;
    string type;
    int user_pref;

    int numberOfSequences;
    int goalModifier[3];// for modifying servo' goal and speed based on user's preferences
    int speedModifier[3];
    vector<SequenceOfActions> sequence_of_actions;

};

class BehaviourParser {

public:
    BehaviourParser(string xmlPath);
    ~BehaviourParser(){}

    void parseXML(Behaviour *behaviour, char *xml_filename); //xml in default folder
    void parseXML(Behaviour *behaviour, char *xml_filename, char *xml_path); //full path to xml

    void resetBehaviour();
    bool getSequenceOfActions(Behaviour *behaviour, char *actions_array, int *action_array_size);  //read a selected tune
    void getActions(SequenceOfActions *sequence_of_actions, char * actionArray, int *actionArraySize); //read a sequence of tune

    void requestBehaviour(char *xml_filename)
    {
        resetBehaviour(); //reset to play new behaviour
        ROS_INFO("Request behaviour: [%s]", xml_filename); //filename
        parseXML(&requested_behaviour_, xml_filename); //initiliased the request behaviour
    }

    bool getRequestedBehaviour(char * tonesArray, int *tonesSize)
    {
        if (getSequenceOfActions(&requested_behaviour_, tonesArray, tonesSize))
        {
            return true;
        }
        return false;
    }

private:
    string default_xml_path_;
    Behaviour requested_behaviour_;
    int sequence_counter_;
};

BehaviourParser::BehaviourParser (string default_xml_path) :  default_xml_path_(default_xml_path)
{
    sequence_counter_=999;
}

/*
 * Open a tune xml file to read the its contents and set up the tune
 */
void BehaviourParser::parseXML(Behaviour *behaviour , char *xml_filename)
{
  char xml_path[120];
  strcpy(xml_path, default_xml_path_.c_str());
  parseXML(behaviour, xml_filename, xml_path);
}

void BehaviourParser::parseXML(Behaviour *behaviour, char *xml_filename, char *xml_path)
{
  ifstream *ifs;
  string line;
  string buf;
  char *doc;

  // Strings used as XML tags

  // Behaviour node's tags
  string bhv_id, bhv_type, bhv_user_pref;
  bhv_id = "id"; bhv_type = "type"; bhv_user_pref = "userPreference";

  // Channel node's tags

  // Head
  string mg1, mg2, mg3, ms1, ms2, ms3;
  mg1="mg1"; mg1="mg2"; mg1="mg3";
  ms1="ms1"; ms1="ms2"; ms1="ms3";
    // Sequence node's tags
  string goal, speed, md;
  goal="goal"; speed="speed"; md="md";

  //Tunes
  string m_msec1, m_msec2, m_msec3, m_note1, m_note2, m_note3;
  m_msec1 = "m_msec1";  m_msec2 = "m_msec2";  m_msec3 = "m_msec3";
  m_note1 = "m_note1";  m_note2 = "m_note2";  m_note3 = "m_note3";
  
  // Sequence node's tags
  string msec, note, modifier;
  msec = "msec";  note = "note";  modifier = "md";

  SequenceOfActions *sequence_of_actions;
  Action *action;

  char xml_fullpath_filename[150];
  strcpy (xml_fullpath_filename, xml_path);   //get path to the XML folders
  strcat (xml_fullpath_filename, xml_filename);

  ifs = new ifstream(xml_fullpath_filename);

  ROS_INFO("I heard mode: [%s]", xml_fullpath_filename);

  if(!ifs->good())
  {
    printf("Invalid filename: %s\n", xml_fullpath_filename);
    exit(1);
  }
  
  while(ifs->good())
  { 
    while(getline(*ifs, line))  //copy xml contents into buf
      buf += line;
  
    doc = new char[buf.length()+1];
    strcpy(doc, buf.c_str());

    xml_document<> behaviour_xml;
    behaviour_xml.parse<0>(doc);

    //Behaviour node
    xml_node<> *behaviour_node = behaviour_xml.first_node();
    //Behaviour attributes
    for(xml_attribute<> *attribute = behaviour_node->first_attribute(); attribute; attribute = attribute->next_attribute())
    {
          if( !bhv_id.compare(attribute->name()) )
            behaviour->id = attribute->value();
          else if( !bhv_type.compare(attribute->name()) )
            behaviour->type = attribute->value();
          else if( !bhv_user_pref.compare(attribute->name()) )
            behaviour->user_pref = atoi(attribute->value());
    }

    //Channel node
    for(xml_node<> *channel_node = behaviour_node->first_node(); channel_node; channel_node = channel_node->next_sibling()) //each expressive channel
    {
        //Channel attributes
        for( xml_attribute<> *attribute = channel_node->first_attribute(); attribute; attribute = attribute->next_attribute())
        {
            if( !mg1.compare(attribute->name()) )
                behaviour->goalModifier[0] = atoi(attribute->value());
            else if( !mg2.compare(attribute->name()) )
                behaviour->goalModifier[1] = atoi(attribute->value());
            else if( !mg3.compare(attribute->name()) )
                behaviour->goalModifier[2] = atoi(attribute->value());
            else if( !ms1.compare(attribute->name()) )
                behaviour->speedModifier[0] = atoi(attribute->value());
            else if( !ms2.compare(attribute->name()) )
                behaviour->speedModifier[1] = atoi(attribute->value());
            else if( !ms3.compare(attribute->name()) )
                behaviour->speedModifier[2] = atoi(attribute->value());
        }

        if (behaviour->sequence_of_actions.size()!=0) //clear its contents if it's not empty
            behaviour->sequence_of_actions.erase(behaviour->sequence_of_actions.begin(),behaviour->sequence_of_actions.end());

        //Sequence node
        int i=0;
        for(xml_node<> *sequence_node = channel_node->first_node(); sequence_node; sequence_node = sequence_node->next_sibling()) //each action sequence
        {
          sequence_of_actions = new SequenceOfActions;                     //create a new sequence container
          behaviour->sequence_of_actions.push_back(*sequence_of_actions);   //add the container as a new sequence

          //Action node
          int j=0;
          for(xml_node<> *action_node = sequence_node->first_node(); action_node; action_node = action_node->next_sibling())  //each action
          {
            action = new Action;  //create a new joint container
            behaviour->sequence_of_actions[i].action.push_back(*action); //add the action container

            //Action attributes
            for(xml_attribute<> *attribute = action_node->first_attribute(); attribute; attribute = attribute->next_attribute())
            { //read each attribute of an action one by one
              if( !goal.compare(attribute->name()) )
                behaviour->sequence_of_actions[i].action[j].goal = atoi(attribute->value());
              else if( !speed.compare(attribute->name()) )
                behaviour->sequence_of_actions[i].action[j].param = atoi(attribute->value());
              else if( !md.compare(attribute->name()) )
                behaviour->sequence_of_actions[i].action[j].modifier = atoi(attribute->value());
            }
            j++;  //proceed to next joint/action
          }
          behaviour->sequence_of_actions[i].numberOfActions = j;
          i++; //proceed to next sequence
        }
        behaviour->numberOfSequences = i;
      }
    ifs->close();
  }
}

/******************************************************************************
*   The method is call to reset/reinitialise the behaviour so that a new
*   action sequence can be executed.
*   @author Kheng Lee Koay
*   @param
*   @date 22/12/2011
******************************************************************************/
void BehaviourParser::resetBehaviour()
{
    sequence_counter_=0;
}
/******************************************************************************
*   The method is use to execute the midi tune behaviour. It can handle a
*   sequence or multiple sequences of midi tune. It will complete a sequence
*   before proceeding to the next sequence, unless a resetBehaviour() is call
*   prior to executing a new tune behaviour. This function should be called from
*   within a loop or in a thread where it get call all the time so that it have
*   the opportunity to execute all the sequences of a tune behaviour.
*
*
*   @author Kheng Lee Koay
*   @param 
*   @date 22/12/2011
******************************************************************************/
bool BehaviourParser::getSequenceOfActions(Behaviour *behaviour, char *actions_array, int *action_array_size)
{

  if(sequence_counter_ < behaviour->sequence_of_actions.size()) //play a sequence of actions
  {
    ROS_INFO("sequence_counter [%d]", sequence_counter_);
    getActions(&behaviour->sequence_of_actions[sequence_counter_], actions_array, action_array_size);
    sequence_counter_++;
    return true;
  }
  return false;
}

/******************************************************************************
*   The method compose the sequence of tune and send it to the driver.
*   @author Kheng Lee Koay
*   @param 
*   @param requestCheck 
*   @date 22/12/2011

******************************************************************************/
void BehaviourParser::getActions(SequenceOfActions *sequence_of_actions, char *actions_array, int *action_array_size)
{

  char temp_array[40];
  int temp_array_size =0;

  for(int i=0; i< sequence_of_actions->action.size(); i++)  {
    temp_array[i*2] = sequence_of_actions->action[i].goal;
    temp_array[i*2+1] = sequence_of_actions->action[i].param;
  }	     

  temp_array_size = sequence_of_actions->action.size();

  *action_array_size = temp_array_size; //passing back the size

  for(int i=0; i<temp_array_size; i++)
  {
    actions_array[i*2] = temp_array[i*2]; //and the tone
    actions_array[i*2+1] = temp_array[i*2+1]; //and the tone
  }
}

#endif
