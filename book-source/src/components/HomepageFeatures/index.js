import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Robotic Nervous System (ROS 2)',
    icon: '🤖',
    description: (
      <>
        Build robot control systems using ROS 2, the industry-standard middleware for robotics applications.
      </>
    ),
    linkTo: '/docs/ros2-fundamentals',
  },
  {
    title: 'Digital Twin (Simulation)',
    icon: '🎮',
    description: (
      <>
        Master Gazebo and Unity simulations to test robots virtually before real-world deployment.
      </>
    ),
    linkTo: '/docs/gazebo-simulation',
  },
  {
    title: 'AI-Robot Brain (NVIDIA Isaac)',
    icon: '🧠',
    description: (
      <>
        Integrate AI workflows and perception systems using NVIDIA Isaac for intelligent robot behavior.
      </>
    ),
    linkTo: '/docs/nvidia-isaac-platform',
  },
  {
    title: 'Vision-Language-Action (VLA)',
    icon: '👁️',
    description: (
      <>
        Enable robots to understand natural language commands and translate them into physical actions.
      </>
    ),
    linkTo: '/docs/introduction-to-vla-systems',
  },
  {
    title: 'AI Learning Assistant',
    icon: '💬',
    description: (
      <>
        Ask questions about any topic and get instant answers with citations from the textbook.
      </>
    ),
    linkTo: '/docs/intro',
  },
];

function Feature({icon, title, description, linkTo}) {
  return (
    <div className={clsx('col col--4')}>
      <Link to={linkTo} className={styles.featureLink}>
        <div className="text--center">
          <span className={styles.featureIcon} role="img" aria-label={title}>
            {icon}
          </span>
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.featureHeading}>Key Features</h2>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
