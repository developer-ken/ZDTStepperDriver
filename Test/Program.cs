using System.IO.Ports;
using ZDTStepperDriver;

SerialPort sp = new SerialPort("COM4");
sp.BaudRate = 115200;
sp.Open();
ProtocolLayer port = new ProtocolLayer(sp, alwaysWaitForAck: false);
port.DriverEnable();
while (true)
{
    port.SetMotorSpeed(1, -1000, 0);
    Thread.Sleep(1000);
    port.SetMotorSpeed(1, 1000, 0);
    Thread.Sleep(1000);
}