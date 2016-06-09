#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"

//using FreePIE.Core.Contracts;
//using FreePIE.Core.Plugins.Globals;
//using FreePIE.Core.Plugins.PSMove;
//using FreePIE.Core.Plugins.SensorFusion;
//using FreePIE.Core.Plugins.Strategies;


using System;

namespace FreePIE.Core.Contracts
{
	[AttributeUsage(AttributeTargets.Class)]
	public class GlobalType : Attribute
	{
		public Type Type{ get; set; }

		public bool IsIndexed{ get; set; }
	}

	[AttributeUsage(AttributeTargets.Class)]
	public class Global : Attribute
	{
		public string Name{ get; set; }
	}

	[AttributeUsage(AttributeTargets.Enum)]
	public class GlobalEnum : Attribute { }
}



namespace PSMoveServicePlugin
{
	[GlobalType(Type = typeof(PSMoveGlobal), IsIndexed = true)]
	public class PSMoveServicePlugin : IPlugin
	{
		public object CreateGlobal()
		{
			return new MyPluginGlobal(this);
		}

		public Action Start()
		{
			//This method is called just before script starts, here you can do your start up stuff
			return null;
		}

		public void Stop()
		{
			//Clean up here
		}

		public event EventHandler Started;

		public string FriendlyName
		{
			get{ return "A name that will be used from GUI"; }
		}

		public bool GetProperty(int index, IPluginProperty property)
			{
			}

			public bool SetProperties(Dictionary<string, object> properties)
			{
			}

			public void DoBeforeNextExecute()
			{
				//This method will be executed each iteration of the script
			}
	}

	public class PSMoveGlobalHolder : IUpdatable
	{
		/*
		private readonly Quaternion quaternion = new Quaternion();
		private readonly MoveButtonHandler buttonHandler = new MoveButtonHandler();
		public MoveButtonHandler ButtonHandler{ get{ return buttonHandler; } }

		private readonly MoveButtonHandler navButtonHandler = new MoveButtonHandler();
		public MoveButtonHandler NavButtonHandler{ get{ return navButtonHandler; } }

			public PSMoveGlobalHolder()
		{
			Global = new PSMoveGlobal(this);
		}

		public void Update()
		{
			if (NewData)
				OnUpdate();
		}

		public bool NewData{ get; set; }
		public PSMoveGlobal Global{ get; private set; }
		public Action OnUpdate{ get; set; }
		public bool GlobalHasUpdateListener{ get; set; }

			public void MoveUpdate(Api.Vector3 position, Api.Quaternion q, int trigger)
		{
			Global.x = position.x;
			Global.y = position.y;
			Global.z = position.z;

			quaternion.Update(q.w, q.x, q.y, q.z);

			Global.yaw = quaternion.Pitch;
			Global.pitch = quaternion.Roll;
			Global.roll = quaternion.Yaw;

			Global.trigger = trigger;

			//Global.battery = battery;

			NewData = true;
		}

		public void NavUpdate(int trigger1, int trigger2, int stickX, int stickY)
		{
			Global.trigger1 = trigger1;
			Global.trigger2 = trigger2;
			Global.stickX = stickX;
			Global.stickY = stickY;
		}*/
	}

	[Global(Name = "psmove")]
	public class PSMoveGlobal : UpdateblePluginGlobal<PSMoveGlobalHolder>
	{
		public PSMoveGlobal(PSMoveGlobalHolder plugin) : base(plugin) { }

		private readonly MyPlugin myPlugin;

		public MyPluginGlobal(MyPlugin myPlugin)
		{
			this.myPlugin = myPlugin;
		}

		public void setYaw(double yaw)
		{
			//Call some method on plugin to set yaw
		}
	}
}