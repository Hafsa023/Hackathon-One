import React, { ReactNode } from 'react';

type CalloutType = 'info' | 'success' | 'warning' | 'danger';

interface CalloutProps {
  type?: CalloutType;
  title?: string;
  children: ReactNode;
}

const icons: Record<CalloutType, string> = {
  info: 'info',
  success: 'check_circle',
  warning: 'warning',
  danger: 'error',
};

const emojis: Record<CalloutType, string> = {
  info: 'i',
  success: '!',
  warning: '!',
  danger: '!',
};

export default function Callout({
  type = 'info',
  title,
  children,
}: CalloutProps): JSX.Element {
  return (
    <div className={`pai-callout pai-callout--${type}`}>
      <div className="pai-callout__icon">
        {emojis[type]}
      </div>
      <div className="pai-callout__content">
        {title && <div className="pai-callout__title">{title}</div>}
        <div className="pai-callout__body">{children}</div>
      </div>
    </div>
  );
}
