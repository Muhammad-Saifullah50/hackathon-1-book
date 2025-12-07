import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';
import curriculumData from './src/data/curriculum.json';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    ...curriculumData.map(module => ({
      type: 'category',
      label: module.title,
      link: {
        type: 'doc',
        id: `${module.slug.substring(1)}/overview`,
      },
      items: module.lessons.map(lesson => ({
        type: 'doc',
        id: lesson.slug.substring(1),
        label: lesson.title,
      })),
    })),
  ],
};

export default sidebars;