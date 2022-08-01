/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.Swc
{
    public class Control : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "swc_msgs/Control";

        //  Target motor RPM (rpm)
        public float rpm;
        //  Target rudder fin angle (degrees)
        public float turn_angle;

        public Control()
        {
            this.rpm = 0.0f;
            this.turn_angle = 0.0f;
        }

        public Control(float rpm, float turn_angle)
        {
            this.rpm = rpm;
            this.turn_angle = turn_angle;
        }
    }
}