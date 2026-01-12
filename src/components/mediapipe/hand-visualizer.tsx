// Hand landmark visualization overlay

import { useRef, useEffect, memo } from 'react'
import type { ProcessedHandData, NormalizedLandmark } from '@/types/mediapipe'
import { LANDMARK } from '@/types/mediapipe'

interface HandVisualizerProps {
  handData: ProcessedHandData | null
  width: number
  height: number
  showConnections?: boolean
  showLandmarks?: boolean
  landmarkColor?: string
  connectionColor?: string
  className?: string
}

// Finger connections for drawing
const FINGER_CONNECTIONS: [number, number][] = [
  // Thumb
  [LANDMARK.WRIST, LANDMARK.THUMB_CMC],
  [LANDMARK.THUMB_CMC, LANDMARK.THUMB_MCP],
  [LANDMARK.THUMB_MCP, LANDMARK.THUMB_IP],
  [LANDMARK.THUMB_IP, LANDMARK.THUMB_TIP],
  // Index
  [LANDMARK.WRIST, LANDMARK.INDEX_MCP],
  [LANDMARK.INDEX_MCP, LANDMARK.INDEX_PIP],
  [LANDMARK.INDEX_PIP, LANDMARK.INDEX_DIP],
  [LANDMARK.INDEX_DIP, LANDMARK.INDEX_TIP],
  // Middle
  [LANDMARK.WRIST, LANDMARK.MIDDLE_MCP],
  [LANDMARK.MIDDLE_MCP, LANDMARK.MIDDLE_PIP],
  [LANDMARK.MIDDLE_PIP, LANDMARK.MIDDLE_DIP],
  [LANDMARK.MIDDLE_DIP, LANDMARK.MIDDLE_TIP],
  // Ring
  [LANDMARK.WRIST, LANDMARK.RING_MCP],
  [LANDMARK.RING_MCP, LANDMARK.RING_PIP],
  [LANDMARK.RING_PIP, LANDMARK.RING_DIP],
  [LANDMARK.RING_DIP, LANDMARK.RING_TIP],
  // Pinky
  [LANDMARK.WRIST, LANDMARK.PINKY_MCP],
  [LANDMARK.PINKY_MCP, LANDMARK.PINKY_PIP],
  [LANDMARK.PINKY_PIP, LANDMARK.PINKY_DIP],
  [LANDMARK.PINKY_DIP, LANDMARK.PINKY_TIP],
  // Palm
  [LANDMARK.INDEX_MCP, LANDMARK.MIDDLE_MCP],
  [LANDMARK.MIDDLE_MCP, LANDMARK.RING_MCP],
  [LANDMARK.RING_MCP, LANDMARK.PINKY_MCP],
]

export const HandVisualizer = memo(function HandVisualizer({
  handData,
  width,
  height,
  showConnections = true,
  showLandmarks = true,
  landmarkColor = '#FF5F00',
  connectionColor = '#00FF00',
  className = '',
}: HandVisualizerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Clear canvas
    ctx.clearRect(0, 0, width, height)

    if (!handData || !handData.landmarks || handData.landmarks.length === 0) {
      return
    }

    const landmarks = handData.landmarks

    // Draw connections
    if (showConnections) {
      ctx.strokeStyle = connectionColor
      ctx.lineWidth = 2
      ctx.lineCap = 'round'

      for (const [start, end] of FINGER_CONNECTIONS) {
        const startLandmark = landmarks[start]
        const endLandmark = landmarks[end]

        if (startLandmark && endLandmark) {
          ctx.beginPath()
          ctx.moveTo(startLandmark.x * width, startLandmark.y * height)
          ctx.lineTo(endLandmark.x * width, endLandmark.y * height)
          ctx.stroke()
        }
      }
    }

    // Draw landmarks
    if (showLandmarks) {
      for (let i = 0; i < landmarks.length; i++) {
        const landmark = landmarks[i]
        const x = landmark.x * width
        const y = landmark.y * height

        // Draw outer circle
        ctx.beginPath()
        ctx.arc(x, y, 6, 0, 2 * Math.PI)
        ctx.fillStyle = landmarkColor
        ctx.fill()

        // Draw inner circle
        ctx.beginPath()
        ctx.arc(x, y, 3, 0, 2 * Math.PI)
        ctx.fillStyle = '#FFFFFF'
        ctx.fill()
      }

      // Highlight fingertips with larger circles
      const fingertips = [
        LANDMARK.THUMB_TIP,
        LANDMARK.INDEX_TIP,
        LANDMARK.MIDDLE_TIP,
        LANDMARK.RING_TIP,
        LANDMARK.PINKY_TIP,
      ]

      for (const tip of fingertips) {
        const landmark = landmarks[tip]
        if (landmark) {
          ctx.beginPath()
          ctx.arc(landmark.x * width, landmark.y * height, 8, 0, 2 * Math.PI)
          ctx.strokeStyle = '#FFFFFF'
          ctx.lineWidth = 2
          ctx.stroke()
        }
      }
    }

    // Draw pinch indicator
    if (handData.pinchDistance < 0.3) {
      const thumbTip = landmarks[LANDMARK.THUMB_TIP]
      const indexTip = landmarks[LANDMARK.INDEX_TIP]

      if (thumbTip && indexTip) {
        const midX = (thumbTip.x + indexTip.x) / 2 * width
        const midY = (thumbTip.y + indexTip.y) / 2 * height
        const radius = 15 + (1 - handData.pinchDistance) * 10

        ctx.beginPath()
        ctx.arc(midX, midY, radius, 0, 2 * Math.PI)
        ctx.strokeStyle = handData.pinchDistance < 0.1 ? '#FF0000' : '#FFFF00'
        ctx.lineWidth = 3
        ctx.stroke()
      }
    }
  }, [handData, width, height, showConnections, showLandmarks, landmarkColor, connectionColor])

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      className={`pointer-events-none ${className}`}
      style={{ position: 'absolute', top: 0, left: 0 }}
    />
  )
})
