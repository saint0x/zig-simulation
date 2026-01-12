// Low-pass filter for smoothing values over time

export class LowPassFilter {
  private value: number
  private alpha: number
  private initialized: boolean = false

  constructor(smoothingFactor: number = 0.3) {
    this.alpha = Math.max(0, Math.min(1, smoothingFactor))
    this.value = 0
  }

  update(newValue: number): number {
    if (!this.initialized) {
      this.value = newValue
      this.initialized = true
      return newValue
    }

    // Exponential moving average
    this.value = this.alpha * newValue + (1 - this.alpha) * this.value
    return this.value
  }

  getValue(): number {
    return this.value
  }

  reset(value?: number): void {
    this.initialized = value !== undefined
    this.value = value ?? 0
  }

  setAlpha(alpha: number): void {
    this.alpha = Math.max(0, Math.min(1, alpha))
  }
}

// Multi-value low-pass filter
export class MultiLowPassFilter {
  private filters: Map<string, LowPassFilter> = new Map()
  private alpha: number

  constructor(smoothingFactor: number = 0.3) {
    this.alpha = smoothingFactor
  }

  update(key: string, value: number): number {
    if (!this.filters.has(key)) {
      this.filters.set(key, new LowPassFilter(this.alpha))
    }
    return this.filters.get(key)!.update(value)
  }

  updateAll(values: Record<string, number>): Record<string, number> {
    const result: Record<string, number> = {}
    for (const [key, value] of Object.entries(values)) {
      result[key] = this.update(key, value)
    }
    return result
  }

  getValue(key: string): number | undefined {
    return this.filters.get(key)?.getValue()
  }

  reset(): void {
    this.filters.clear()
  }

  setAlpha(alpha: number): void {
    this.alpha = alpha
    for (const filter of this.filters.values()) {
      filter.setAlpha(alpha)
    }
  }
}
