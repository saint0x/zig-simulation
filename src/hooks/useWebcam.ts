// Webcam access and stream management hook

import { useState, useRef, useCallback, useEffect } from 'react'
import type { UseWebcamReturn } from '@/types/mediapipe'

interface WebcamConfig {
  width?: number
  height?: number
  facingMode?: 'user' | 'environment'
}

export function useWebcam(config: WebcamConfig = {}): UseWebcamReturn {
  const [isStreaming, setIsStreaming] = useState(false)
  const [error, setError] = useState<Error | null>(null)

  const videoRef = useRef<HTMLVideoElement>(null)
  const streamRef = useRef<MediaStream | null>(null)

  const { width = 640, height = 480, facingMode = 'user' } = config

  const startStream = useCallback(async () => {
    if (streamRef.current) {
      return // Already streaming
    }

    setError(null)

    try {
      const constraints: MediaStreamConstraints = {
        video: {
          width: { ideal: width },
          height: { ideal: height },
          facingMode,
          frameRate: { ideal: 30 },
        },
        audio: false,
      }

      const stream = await navigator.mediaDevices.getUserMedia(constraints)
      streamRef.current = stream

      if (videoRef.current) {
        videoRef.current.srcObject = stream
        await videoRef.current.play()
        setIsStreaming(true)
        console.log('Webcam stream started')
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to access webcam')
      setError(error)
      console.error('Webcam error:', error)
      throw error
    }
  }, [width, height, facingMode])

  const stopStream = useCallback(() => {
    if (streamRef.current) {
      streamRef.current.getTracks().forEach(track => track.stop())
      streamRef.current = null
    }

    if (videoRef.current) {
      videoRef.current.srcObject = null
    }

    setIsStreaming(false)
    console.log('Webcam stream stopped')
  }, [])

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      stopStream()
    }
  }, [stopStream])

  return {
    videoRef,
    isStreaming,
    error,
    startStream,
    stopStream,
  }
}
