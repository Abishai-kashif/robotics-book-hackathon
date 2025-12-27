import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import styles from './styles.module.css';

// Client-side only component using Spline Runtime directly
const SplineClientComponent = () => {
  const canvasRef = React.useRef(null);
  const [isLoading, setIsLoading] = React.useState(true);
  const [error, setError] = React.useState(null);

  React.useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    let splineApp = null;
    let mounted = true;

    async function loadSpline() {
      try {
        // Load Spline Runtime directly (more reliable for SSR contexts)
        const { Application } = await import('@splinetool/runtime');

        if (!mounted || !canvasRef.current) return;

        // Create canvas element
        const canvas = document.createElement('canvas');
        canvas.id = 'spline-canvas-3d';
        canvas.style.width = '100%';
        canvas.style.height = '100%';
        canvas.className = styles.splineCanvas;

        // Clear and append canvas
        canvasRef.current.innerHTML = '';
        canvasRef.current.appendChild(canvas);

        // Initialize Spline
        splineApp = new Application(canvas);
        await splineApp.load('https://prod.spline.design/YBKvHlKjcfCIMj5G/scene.splinecode');

        if (mounted) {
          setIsLoading(false);
        }
      } catch (err) {
        console.error('Failed to load Spline:', err);
        if (mounted) {
          setError(err.message);
          setIsLoading(false);
        }
      }
    }

    loadSpline();

    return () => {
      mounted = false;
      if (splineApp) {
        // Cleanup if Spline provides a dispose method
        try {
          splineApp.dispose?.();
        } catch (e) {
          console.warn('Spline cleanup warning:', e);
        }
      }
    };
  }, []);

  if (error) {
    return (
      <div className={styles.loadingPlaceholder}>
        <p style={{ color: 'var(--ifm-color-danger)' }}>
          Failed to load 3D scene
        </p>
      </div>
    );
  }

  return (
    <div className={styles.splineWrapper}>
      {isLoading && (
        <div className={styles.loadingPlaceholder}>
          <div className={styles.loadingSpinner}></div>
          <p>Loading 3D Scene...</p>
        </div>
      )}
      <div
        ref={canvasRef}
        className={styles.splineCanvasContainer}
        style={{ opacity: isLoading ? 0 : 1 }}
      />
    </div>
  );
};

export default function RobotCharacter3D() {
  return (
    <div className={styles.splineContainer}>
      <BrowserOnly
        fallback={
          <div className={styles.loadingPlaceholder}>
            <div className={styles.loadingSpinner}></div>
            <p>Loading 3D Scene...</p>
          </div>
        }
      >
        {() => <SplineClientComponent />}
      </BrowserOnly>
    </div>
  );
}
