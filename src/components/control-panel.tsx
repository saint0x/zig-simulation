"use client"
import { Slider } from "@/components/ui/slider"
import { Card, CardContent, CardTitle, CardHeader } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { Badge } from "@/components/ui/badge"

export function ControlPanel({ angles, onAngleChange, onMovementSelect, controlSignals }) {
  return (
    <div className="absolute bottom-4 left-4 right-4 md:right-auto md:w-96 bg-black/80 backdrop-blur-md rounded-lg text-white overflow-y-auto max-h-[calc(100vh-8rem)]">
      <Card className="border-0 bg-transparent text-white">
        <CardHeader className="pb-2">
          <div className="flex items-center justify-between">
            <CardTitle className="text-xl font-mono">KUKA Robot Control</CardTitle>
            <Badge
              variant="outline"
              className={`${
                controlSignals.systemStatus === "READY"
                  ? "bg-green-600/20 text-green-400 border-green-600"
                  : "bg-yellow-600/20 text-yellow-400 border-yellow-600"
              }`}
            >
              {controlSignals.systemStatus}
            </Badge>
          </div>
        </CardHeader>
        <CardContent>
          <Tabs defaultValue="manual" className="w-full">
            <TabsList className="w-full bg-black/50 mb-4">
              <TabsTrigger className="text-white data-[state=active]:bg-white/10" value="manual">
                Manual Control
              </TabsTrigger>
              <TabsTrigger className="text-white data-[state=active]:bg-white/10" value="presets">
                Presets
              </TabsTrigger>
              <TabsTrigger className="text-white data-[state=active]:bg-white/10" value="diagnostics">
                Diagnostics
              </TabsTrigger>
            </TabsList>

            <TabsContent value="manual" className="mt-0">
              <div className="space-y-4">
                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Base Rotation</label>
                    <span className="text-sm font-mono">{angles.baseRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles?.baseRotation ?? 0]}
                    min={-180}
                    max={180}
                    step={1}
                    onValueChange={([value]) => onAngleChange("baseRotation", value)}
                    className="cursor-pointer"
                  />
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Shoulder</label>
                    <span className="text-sm font-mono">{angles.shoulderRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles.shoulderRotation]}
                    min={-120}
                    max={30}
                    step={1}
                    onValueChange={([value]) => onAngleChange("shoulderRotation", value)}
                    className="cursor-pointer"
                  />
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Elbow</label>
                    <span className="text-sm font-mono">{angles.elbowRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles.elbowRotation]}
                    min={-120}
                    max={120}
                    step={1}
                    onValueChange={([value]) => onAngleChange("elbowRotation", value)}
                    className="cursor-pointer"
                  />
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Wrist Bend</label>
                    <span className="text-sm font-mono">{angles.wristBendRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles.wristBendRotation]}
                    min={-100}
                    max={100}
                    step={1}
                    onValueChange={([value]) => onAngleChange("wristBendRotation", value)}
                    className="cursor-pointer"
                  />
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Wrist Rotation</label>
                    <span className="text-sm font-mono">{angles.wristRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles.wristRotation]}
                    min={-180}
                    max={180}
                    step={1}
                    onValueChange={([value]) => onAngleChange("wristRotation", value)}
                    className="cursor-pointer"
                  />
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Tool Rotation</label>
                    <span className="text-sm font-mono">{angles.toolRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles.toolRotation]}
                    min={-180}
                    max={180}
                    step={1}
                    onValueChange={([value]) => onAngleChange("toolRotation", value)}
                    className="cursor-pointer"
                  />
                </div>

                <div className="space-y-2">
                  <div className="flex justify-between">
                    <label className="text-sm font-medium">Gripper</label>
                    <span className="text-sm font-mono">{angles.gripperRotation.toFixed(0)}°</span>
                  </div>
                  <Slider
                    value={[angles.gripperRotation]}
                    min={-90}
                    max={0}
                    step={1}
                    onValueChange={([value]) => onAngleChange("gripperRotation", value)}
                    className="cursor-pointer"
                  />
                </div>
              </div>
            </TabsContent>

            <TabsContent value="presets" className="mt-0">
              <div className="grid grid-cols-1 gap-2">
                <Button onClick={() => onMovementSelect("rest")} variant="outline" className="justify-start font-mono">
                  Home Position
                </Button>
                <Button
                  onClick={() => onMovementSelect("pickup")}
                  variant="outline"
                  className="justify-start font-mono"
                >
                  Pick Up Object
                </Button>
                <Button onClick={() => onMovementSelect("place")} variant="outline" className="justify-start font-mono">
                  Place Object
                </Button>
                <Button
                  onClick={() => onMovementSelect("inspect")}
                  variant="outline"
                  className="justify-start font-mono"
                >
                  Inspection Position
                </Button>
                <Button
                  onClick={() => onMovementSelect("grasp")}
                  variant="outline"
                  className="justify-start font-mono text-orange-500 hover:text-orange-400"
                >
                  Grasp Object
                </Button>
              </div>
            </TabsContent>

            <TabsContent value="diagnostics" className="mt-0">
              <div className="space-y-4">
                <div>
                  <h3 className="text-sm font-semibold mb-2">Motor Current (A)</h3>
                  <div className="grid grid-cols-7 gap-1">
                    {(controlSignals.motorCurrents || [0,0,0,0,0,0,0]).map((current, i) => (
                      <div key={i} className="flex flex-col items-center">
                        <div className="text-xs text-gray-400">J{i + 1}</div>
                        <div className="text-sm font-mono">{(current || 0).toFixed(2)}</div>
                      </div>
                    ))}
                  </div>
                </div>

                <div>
                  <h3 className="text-sm font-semibold mb-2">Temperature (°C)</h3>
                  <div className="grid grid-cols-7 gap-1">
                    {(controlSignals.motorTemperatures || controlSignals.sensorReadings || [25,25,25,25,25,25,25]).map((temp, i) => (
                      <div key={i} className="flex flex-col items-center">
                        <div className="text-xs text-gray-400">J{i + 1}</div>
                        <div className={`text-sm font-mono ${(temp || 0) > 60 ? 'text-red-400' : (temp || 0) > 45 ? 'text-yellow-400' : 'text-green-400'}`}>
                          {(temp || 25).toFixed(1)}
                        </div>
                      </div>
                    ))}
                  </div>
                </div>

                <div>
                  <h3 className="text-sm font-semibold mb-2">Control Mode</h3>
                  <Badge variant="outline" className="bg-blue-600/20 text-blue-400 border-blue-600">
                    {controlSignals.controlMode || 'POSITION'}
                  </Badge>
                </div>

                {controlSignals.emergencyStop && (
                  <div className="bg-red-900/50 p-2 rounded border border-red-500">
                    <h3 className="text-sm font-semibold text-red-400">EMERGENCY STOP ACTIVE</h3>
                    <p className="text-xs text-red-300 mt-1">
                      System halted. Reset required to continue operation.
                    </p>
                  </div>
                )}

                {controlSignals.collisionDetected && (
                  <div className="bg-orange-900/30 p-2 rounded border border-orange-700">
                    <h3 className="text-sm font-semibold text-orange-400">Collision Detected</h3>
                    <p className="text-xs text-orange-300 mt-1">
                      Robot stopped due to collision detection.
                    </p>
                  </div>
                )}

                {controlSignals.errorCode && (
                  <div className="bg-red-900/30 p-2 rounded border border-red-700">
                    <h3 className="text-sm font-semibold text-red-400">Error: {controlSignals.errorCode}</h3>
                    <p className="text-xs text-red-300 mt-1">
                      {controlSignals.errorCode === "TEMP_HIGH"
                        ? "Motor temperature exceeding normal range. Check cooling system."
                        : controlSignals.errorCode === "FAULT"
                        ? "System fault detected. Check robot state."
                        : "Unknown error. Contact maintenance."}
                    </p>
                  </div>
                )}
              </div>
            </TabsContent>
          </Tabs>

          <div className="mt-4 pt-4 border-t border-white/20 text-xs text-white/60">
            <p>Camera: Click and drag to orbit. Scroll to zoom.</p>
            <p className="mt-1">
              Zig backend status:{" "}
              <span className={
                controlSignals.systemStatus === "READY" || controlSignals.systemStatus === "BUSY"
                  ? "text-green-400"
                  : controlSignals.systemStatus === "WARNING"
                  ? "text-yellow-400"
                  : controlSignals.systemStatus === "ERROR"
                  ? "text-red-400"
                  : "text-gray-400"
              }>
                {controlSignals.systemStatus === "READY" || controlSignals.systemStatus === "BUSY"
                  ? "Connected"
                  : controlSignals.systemStatus}
              </span>
            </p>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

