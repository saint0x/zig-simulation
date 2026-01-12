// Hand tracking control panel UI

import { useState, useCallback } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Slider } from '@/components/ui/slider'
import { Switch } from '@/components/ui/switch'
import { Label } from '@/components/ui/label'
import { HandTracker } from './hand-tracker'
import { Hand, Camera, CameraOff, AlertTriangle, Settings, Activity } from 'lucide-react'
import type {
  JointAngles,
  DiscreteCommand,
  GestureResult,
  ProcessedHandData,
} from '@/types/mediapipe'

interface HandTrackingPanelProps {
  onJointAnglesChange: (angles: JointAngles) => void
  onDiscreteCommand?: (command: DiscreteCommand) => void
  onEStop?: () => void
  className?: string
}

export function HandTrackingPanel({
  onJointAnglesChange,
  onDiscreteCommand,
  onEStop,
  className = '',
}: HandTrackingPanelProps) {
  const [enabled, setEnabled] = useState(false)
  const [isTracking, setIsTracking] = useState(false)
  const [showSettings, setShowSettings] = useState(false)
  const [sensitivity, setSensitivity] = useState(1.0)
  const [smoothing, setSmoothing] = useState(0.3)
  const [showVideo, setShowVideo] = useState(true)
  const [currentGesture, setCurrentGesture] = useState<GestureResult | null>(null)
  const [currentHandData, setCurrentHandData] = useState<ProcessedHandData | null>(null)
  const [lastCommand, setLastCommand] = useState<DiscreteCommand | null>(null)

  const handleToggle = useCallback(() => {
    setEnabled(prev => !prev)
  }, [])

  const handleDiscreteCommand = useCallback((command: DiscreteCommand) => {
    setLastCommand(command)
    onDiscreteCommand?.(command)

    // Clear command indicator after 1 second
    setTimeout(() => setLastCommand(null), 1000)
  }, [onDiscreteCommand])

  const handleEStop = useCallback(() => {
    setEnabled(false)
    onEStop?.()
  }, [onEStop])

  return (
    <Card className={`bg-black/80 backdrop-blur-md border-orange-600/30 ${className}`}>
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Hand className="w-5 h-5 text-orange-400" />
            <CardTitle className="text-lg text-white">Hand Control</CardTitle>
            {isTracking && (
              <Badge variant="outline" className="bg-green-600/20 text-green-400 border-green-600">
                Active
              </Badge>
            )}
            {lastCommand && (
              <Badge variant="outline" className="bg-orange-600/20 text-orange-400 border-orange-600 animate-pulse">
                {lastCommand.type}
              </Badge>
            )}
          </div>
          <div className="flex items-center gap-2">
            <Button
              variant="ghost"
              size="sm"
              onClick={() => setShowSettings(!showSettings)}
              className="text-gray-400 hover:text-white"
            >
              <Settings className="w-4 h-4" />
            </Button>
            <Button
              variant={enabled ? 'destructive' : 'default'}
              size="sm"
              onClick={handleToggle}
              className={enabled ? '' : 'bg-orange-600 hover:bg-orange-700'}
            >
              {enabled ? <CameraOff className="w-4 h-4 mr-1" /> : <Camera className="w-4 h-4 mr-1" />}
              {enabled ? 'Stop' : 'Start'}
            </Button>
          </div>
        </div>
      </CardHeader>

      <CardContent className="space-y-4">
        {/* Hand Tracker Video */}
        <div className="relative rounded-lg overflow-hidden bg-gray-900">
          <HandTracker
            enabled={enabled}
            showVideo={showVideo}
            showVisualization={true}
            controlConfig={{
              sensitivity,
              smoothingFactor: smoothing,
              smoothingEnabled: true,
            }}
            onJointAnglesChange={onJointAnglesChange}
            onDiscreteCommand={handleDiscreteCommand}
            onEStop={handleEStop}
            onTrackingStateChange={setIsTracking}
            onHandDataChange={setCurrentHandData}
            onGestureChange={setCurrentGesture}
            videoWidth={320}
            videoHeight={240}
            className="w-full"
          />

          {!enabled && (
            <div className="absolute inset-0 flex items-center justify-center bg-gray-900/90">
              <div className="text-center text-gray-400">
                <Camera className="w-12 h-12 mx-auto mb-2 opacity-50" />
                <p className="text-sm">Click Start to enable hand tracking</p>
              </div>
            </div>
          )}
        </div>

        {/* Settings Panel */}
        {showSettings && (
          <div className="space-y-4 pt-2 border-t border-gray-700">
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="text-sm text-gray-300">Sensitivity</Label>
                <span className="text-xs text-orange-400">{sensitivity.toFixed(1)}x</span>
              </div>
              <Slider
                value={[sensitivity]}
                onValueChange={([v]) => setSensitivity(v)}
                min={0.1}
                max={2.0}
                step={0.1}
                className="w-full"
              />
            </div>

            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="text-sm text-gray-300">Smoothing</Label>
                <span className="text-xs text-orange-400">{(smoothing * 100).toFixed(0)}%</span>
              </div>
              <Slider
                value={[smoothing]}
                onValueChange={([v]) => setSmoothing(v)}
                min={0}
                max={0.9}
                step={0.05}
                className="w-full"
              />
            </div>

            <div className="flex items-center justify-between">
              <Label className="text-sm text-gray-300">Show Video</Label>
              <Switch
                checked={showVideo}
                onCheckedChange={setShowVideo}
              />
            </div>
          </div>
        )}

        {/* Status Info */}
        {isTracking && currentHandData && (
          <div className="grid grid-cols-2 gap-2 text-xs text-gray-400 pt-2 border-t border-gray-700">
            <div className="flex items-center gap-1">
              <Activity className="w-3 h-3" />
              <span>Hand: {currentHandData.handedness}</span>
            </div>
            <div>Confidence: {(currentHandData.confidence * 100).toFixed(0)}%</div>
            <div>Pinch: {(currentHandData.pinchDistance * 100).toFixed(0)}%</div>
            <div>Grab: {(currentHandData.grabStrength * 100).toFixed(0)}%</div>
          </div>
        )}

        {/* Gesture Reference */}
        <div className="text-xs text-gray-500 pt-2 border-t border-gray-700">
          <div className="font-medium text-gray-400 mb-1">Gesture Commands:</div>
          <div className="grid grid-cols-2 gap-1">
            <span>Fist: Close gripper</span>
            <span>Palm: Open gripper</span>
            <span>Thumb up: Confirm</span>
            <span>Victory: Mode toggle</span>
            <span className="text-red-400 col-span-2 flex items-center gap-1">
              <AlertTriangle className="w-3 h-3" />
              ILY Sign: Emergency Stop
            </span>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
