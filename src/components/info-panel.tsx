import { Card, CardContent, CardTitle, CardHeader } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Info } from "lucide-react"

export function InfoPanel() {
  return (
    <div className="absolute top-4 right-4 w-80 bg-black/80 backdrop-blur-md rounded-lg text-white overflow-y-auto max-h-[calc(100vh-8rem)]">
      <Card className="border-0 bg-transparent text-white">
        <CardHeader className="pb-2">
          <div className="flex items-center justify-between">
            <CardTitle className="text-xl font-mono">KUKA KR QUANTEC</CardTitle>
            <Badge variant="outline" className="bg-orange-600/20 text-orange-400 border-orange-600">
              Industrial
            </Badge>
          </div>
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            <div className="flex items-start gap-2">
              <Info className="h-5 w-5 text-blue-400 mt-0.5 flex-shrink-0" />
              <p className="text-sm text-gray-300">
                This industrial robot arm features 7 degrees of freedom with a Zig-based microcontroller for precise
                control and real-time feedback.
              </p>
            </div>

            <div className="space-y-2">
              <h3 className="text-sm font-semibold text-gray-200">Technical Specifications:</h3>
              <ul className="text-xs space-y-1 text-gray-300">
                <li>• Max Payload: 300 kg</li>
                <li>• Reach: 3.1 meters</li>
                <li>• Position Repeatability: ±0.05 mm</li>
                <li>• 7 Degrees of Freedom</li>
                <li>• Zig-based Microcontroller</li>
                <li>• Industrial-Grade Servo Motors</li>
                <li>• Real-time Feedback System</li>
                <li>• Collision Detection</li>
              </ul>
            </div>

            <div className="space-y-2">
              <h3 className="text-sm font-semibold text-gray-200">Microcontroller Features:</h3>
              <ul className="text-xs space-y-1 text-gray-300">
                <li>• Zig-based firmware for memory safety</li>
                <li>• Real-time control loop at 1kHz</li>
                <li>• Predictable latency for industrial applications</li>
                <li>• Advanced error handling and recovery</li>
                <li>• Integrated safety systems</li>
                <li>• Remote monitoring and diagnostics</li>
              </ul>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

