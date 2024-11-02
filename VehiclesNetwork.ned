//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

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
        @display("bgb=800,600";);  // 设置仿真画布的尺寸

    submodules:
        vehicle[numVehicles]: VehicleModule {
            parameters:
                xmlFilename = default("data.xml");
            @display("i=device/laptop;is=s;p=100+index*40,400;");  // 使用通用图标并缩小尺寸
        }

    connections allowunconnected:
        for i=0..31,for j=0..31{
             vehicle[i].out[j] --> { @display("ls=white,0";);delay = 100ms;} --> vehicle[j].in[i] if i!=j;
        }
}