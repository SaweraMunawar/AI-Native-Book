import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

type FeatureItem = {
  title: string;
  description: ReactNode;
  icon: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Learn Physical AI',
    icon: '🤖',
    description: (
      <>
        Understand the fundamentals of Physical AI and how robots perceive,
        reason, and act in the real world.
      </>
    ),
  },
  {
    title: 'Master ROS 2',
    icon: '🔧',
    description: (
      <>
        Build robot software using ROS 2, the industry-standard middleware
        for robotics development.
      </>
    ),
  },
  {
    title: 'Simulate with Digital Twins',
    icon: '🎮',
    description: (
      <>
        Create virtual robot replicas with Gazebo and NVIDIA Isaac Sim
        for safe development and testing.
      </>
    ),
  },
  {
    title: 'VLA Systems',
    icon: '🧠',
    description: (
      <>
        Explore Vision-Language-Action models that combine perception,
        language understanding, and robot control.
      </>
    ),
  },
];

function Feature({title, description, icon}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
      <div className="text--center padding-horiz--md">
        <div style={{fontSize: '3rem', marginBottom: '1rem'}}>{icon}</div>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Physical AI & Humanoid Robotics - A comprehensive guide to building intelligent robots">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <section className={styles.chapters}>
          <div className="container">
            <Heading as="h2" className="text--center margin-bottom--lg">
              What You'll Learn
            </Heading>
            <div className="row">
              <div className="col col--4">
                <div className={styles.chapterCard}>
                  <h3>Chapter 1-2</h3>
                  <p><strong>Foundations</strong></p>
                  <p>Introduction to Physical AI and humanoid robot anatomy, kinematics, and sensors.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.chapterCard}>
                  <h3>Chapter 3-4</h3>
                  <p><strong>Development</strong></p>
                  <p>ROS 2 fundamentals and digital twin simulation with Gazebo and Isaac Sim.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.chapterCard}>
                  <h3>Chapter 5-6</h3>
                  <p><strong>Advanced Topics</strong></p>
                  <p>Vision-Language-Action systems and a capstone project to build your own system.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
