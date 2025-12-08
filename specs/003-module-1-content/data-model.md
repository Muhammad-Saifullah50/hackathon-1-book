# Data Model: Module 1 Content

## MDX Front Matter

| Field | Type | Required | Description |
|---|---|---|---|
| title | string | Yes | Chapter title |
| sidebar_label | string | No | Short title for sidebar |
| description | string | Yes | SEO description |
| experience_level | 'Beginner' \| 'Advanced' | Yes | For filtering/badges |

## Component Props

### PersonalizationBar

| Prop | Type | Description |
|---|---|---|
| None | - | It manages its own state (Hardware vs Software mode) |

### Card

| Prop | Type | Description |
|---|---|---|
| title | string | Card title |
| icon | ReactNode | Optional icon |
| children | ReactNode | Card content |
| className | string | Additional styles |

### Alert

| Prop | Type | Description |
|---|---|---|
| variant | 'default' \| 'destructive' | Style variant |
| title | string | Alert title |
| children | ReactNode | Alert content |

### Badge

| Prop | Type | Description |
|---|---|---|
| variant | 'default' \| 'secondary' \| 'outline' | Style variant |
| children | ReactNode | Badge text |
