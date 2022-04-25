//--------------- Initialisation //ROS ------------------------------
int main(int argc, char **argv)
{
  //ROSAds_server_node *node = new //ROSAds_server_node();
  return node->main(argc,argv);
}

int RosAds_Interface::main(int argc, char **argv)
{

  //ROS::init(argc, argv, "//ROSAds_server");
  //ROS::NodeHandle n;
  //ROS::NodeHandle nprive("~");

  if(nprive.hasParam("localNetId"))
  {

    nprive.getParam("localNetId", m_localNetId_param);
    AdsSetLocalAddress(m_localNetId_param);
  }
  else {
    //ROS_ERROR("Param localNetId unknown");
    return 0;
  }

  //------------------ Routing to remote PLC --------------------
  if(nprive.hasParam("remoteNetId"))
  {
    nprive.getParam("remoteNetId", m_remoteNetId);
  }
  else {
    //ROS_ERROR("Param remoteNetId unknown");
    return 0;
  }


  if(nprive.hasParam("remoteIpV4"))
  {
    nprive.getParam("remoteIpV4", m_remoteIpV4);
  }
  else {
    //ROS_ERROR("Param remoteIpV4 unknown");
    return 0;
  }


  if(nprive.hasParam("PLCPathFile"))
  {
    nprive.getParam("PLCPathFile", m_PLCFileDefinitionPath);
  }
  else {
    //ROS_ERROR("Param PLCPathFile unknown");
    return 0;
  }


  //ROS_ERROR_STREAM("GO FOR Init Route");
  initRoute();
  //ROS_ERROR("Init Route done");

  try
  {
    m_VariableADS = m_route->GetDeviceAdsVariables();

    bindPLcVar();

    //ROS_INFO("Ready to communicate with the remote PLC via ADS.");
  }
  catch(AdsException e)
  {
    //ROS_INFO_STREAM(e.what());
    //ROS_ERROR("ERROR in mapping alias with ADS");
  }


  //ROS_ads_msgs::ADS  req;
  //ROS_ads_msgs::ADS res;

  name = "data1"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data2"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data3"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data4"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data5"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data6"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data7"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data8"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);

name = "data9"; adsReadValue(req,res);
  //ROS_INFO("%s\t%f",name.c_str(),(float)res.varValues[0]);


  try
  {
    m_VariableADS = m_route->GetDeviceAdsVariables();

    for(std::map<string,string>::iterator it = m_VariableADS.begin(); it != m_VariableADS.end(); ++it)
    {
      //ROS_INFO("%s\t%s",it->first.c_str(),it->second.c_str());
    }
    bindPLcVar();

    //ROS_INFO("Ready to communicate with the remote PLC via ADS.");
  }
  catch(AdsException e)
  {
    //ROS_INFO_STREAM(e.what());

    //ROS_INFO("ERROR in mapping alias with ADS.");
  }


  //ROS::spin();

  return 0;
}
