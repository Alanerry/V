import inet.examples.wireless.hiddennode.Node;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211DimensionalRadioMedium;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;
simple VehicleModule
{
    parameters:
        string xmlFilename = default("data.xml");
    gates:
        input in[32];  // 定义32个输入门
        output out[32];  // 定义32个输出门
}


network VehiclesNetwork
{
    parameters:
        int numVehicles = default(32);  // 定义车辆数量为32
        @display("bgb=800,600"; );  // 设置仿真画布的尺寸

    submodules:
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=773.12,27.784");
        }
        vehicle[numVehicles]: VehicleModule {
            parameters:
                xmlFilename = default("data.xml");
                @display("i=device/laptop;is=s;p=100+index*40,400;");  // 使用通用图标并缩小尺寸
        }

    connections allowunconnected:
        for i=0..31, for j=0..31 {
            vehicle[i].out[j] --> { @display("ls=white,0"; ); delay = 100ms; } --> vehicle[j].in[i] if i!=j;
        }
}
