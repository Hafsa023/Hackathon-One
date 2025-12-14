import React from 'react';

type BadgeVariant = 'ros' | 'gazebo' | 'isaac' | 'unity' | 'python' | 'default';

interface VersionBadgeProps {
  variant?: BadgeVariant;
  children: React.ReactNode;
}

export default function VersionBadge({
  variant = 'default',
  children,
}: VersionBadgeProps): JSX.Element {
  return (
    <span className={`version-badge version-badge--${variant}`}>
      {children}
    </span>
  );
}
