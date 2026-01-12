// Main hand tracking component combining webcam, MediaPipe, and visualization

import { useState, useCallback, useEffect } from 'react'
import { useMediaPipe } from '@/hooks/useMediaPipe'
import { useWebcam } from '@/hooks/useWebcam'
import { useHandTracking } from '@/hooks/useHandTracking'
import { useGestureControl } from '@/hooks/useGestureControl'
import { HandVisualizer } from './hand-visualizer'
import type {
  ProcessedHandData,
  GestureResult,
  JointAngles,
  DiscreteCommand,
  MediaPipeConfig,
  ControlMappingConfig,
} from '@/types/mediapipe'

interface HandTrackerProps {
  enabled?: boolean
  showVideo?: boolean
  showVisualization?: boolean
  mediaConfig?: Partial<MediaPipeConfig>
  controlConfig?: Partial<ControlMappingConfig>
  onJointAnglesChange?: (angles: JointAngles) => void
  onDiscreteCommand?: (command: DiscreteCommand) => void
  onEStop?: () => void
  onTrackingStateChange?: (isTracking: boolean) => void
  onHandDataChange?: (data: ProcessedHandData | null) => void
  onGestureChange?: (gesture: GestureResult | null) => void
  className?: string
  videoWidth?: number
  videoHeight?: number
}

export function HandTracker({
  enabled = false,
  showVideo = true,
  showVisualization = true,
  mediaConfig,
  controlConfig,
  onJointAnglesChange,
  onDiscreteCommand,
  onEStop,
  onTrackingStateChange,
  onHandDataChange,
  onGestureChange,
  className = '',
  videoWidth = 320,
  videoHeight = 240,
}: HandTrackerProps) {
  const [isInitialized, setIsInitialized] = useState(false)

  // Initialize MediaPipe
  const {
    isInitialized: mediaPipeReady,
    isLoading: mediaPipeLoading,
    error: mediaPipeError,
    gestureRecognizer,
  } = useMediaPipe(mediaConfig)

  // Initialize webcam
  const {
    videoRef,
    isStreaming,
    error: webcamError,
    startStream,
    stopStream,
  } = useWebcam({ width: videoWidth, height: videoHeight })

  // Initialize hand tracking
  const {
    isTracking,
    handData,
    gesture,
    fps,
    startTracking,
    stopTracking,
  } = useHandTracking(videoRef, gestureRecognizer, {
    enabled,
    mirrorMode: mediaConfig?.mirrorMode ?? true,
    onLandmarksUpdate: onHandDataChange,
    onGestureDetected: onGestureChange,
  })

  // Initialize gesture control
  const {
    jointAngles,
    discreteCommand,
    isControlling,
    enable: enableControl,
    disable: disableControl,
  } = useGestureControl(handData, gesture, {
    config: controlConfig,
    onJointAnglesChange,
    onDiscreteCommand,
    onEStop,
  })

  // Handle enable/disable
  useEffect(() => {
    if (enabled && mediaPipeReady && !isStreaming) {
      startStream().catch(console.error)
    } else if (!enabled && isStreaming) {
      stopTracking()
      disableControl()
      stopStream()
    }
  }, [enabled, mediaPipeReady, isStreaming, startStream, stopStream, stopTracking, disableControl])

  // Start tracking when stream is ready
  useEffect(() => {
    if (enabled && isStreaming && mediaPipeReady && !isTracking) {
      startTracking()
      enableControl()
      setIsInitialized(true)
    }
  }, [enabled, isStreaming, mediaPipeReady, isTracking, startTracking, enableControl])

  // Notify parent of tracking state changes
  useEffect(() => {
    onTrackingStateChange?.(isTracking)
  }, [isTracking, onTrackingStateChange])

  const error = mediaPipeError || webcamError

  return (
    <div className={`relative ${className}`} style={{ width: videoWidth, height: videoHeight }}>
      {/* Status overlay */}
      {!isInitialized && (
        <div className="absolute inset-0 bg-black/80 flex items-center justify-center z-20">
          <div className="text-center text-white">
            {mediaPipeLoading && <p className="animate-pulse">Loading MediaPipe...</p>}
            {error && <p className="text-red-400">Error: {error.message}</p>}
            {!enabled && !error && <p className="text-gray-400">Hand tracking disabled</p>}
          </div>
        </div>
      )}

      {/* Video element */}
      <video
        ref={videoRef}
        className={`absolute inset-0 w-full h-full object-cover ${showVideo ? '' : 'opacity-0'}`}
        style={{ transform: 'scaleX(-1)' }} // Mirror for user-facing camera
        playsInline
        muted
      />

      {/* Hand visualization overlay */}
      {showVisualization && handData && (
        <HandVisualizer
          handData={handData}
          width={videoWidth}
          height={videoHeight}
          className="z-10"
        />
      )}

      {/* Stats overlay */}
      {isTracking && (
        <div className="absolute top-2 left-2 bg-black/60 text-white text-xs px-2 py-1 rounded z-20">
          <div>FPS: {fps}</div>
          {gesture && gesture.categoryName !== 'None' && (
            <div className="text-orange-400">
              {gesture.categoryName} ({(gesture.score * 100).toFixed(0)}%)
            </div>
          )}
          {handData && (
            <div className="text-green-400">
              Pinch: {(handData.pinchDistance * 100).toFixed(0)}%
            </div>
          )}
        </div>
      )}
    </div>
  )
}
