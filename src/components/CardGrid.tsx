import React, { ReactNode } from 'react';

interface CardProps {
  icon?: string;
  title: string;
  children: ReactNode;
  href?: string;
}

export function Card({ icon, title, children, href }: CardProps): JSX.Element {
  const content = (
    <>
      {icon && <div className="pai-card__icon">{icon}</div>}
      <h4 className="pai-card__title">{title}</h4>
      <div className="pai-card__description">{children}</div>
    </>
  );

  if (href) {
    return (
      <a href={href} className="pai-card" style={{ textDecoration: 'none' }}>
        {content}
      </a>
    );
  }

  return <div className="pai-card">{content}</div>;
}

interface CardGridProps {
  children: ReactNode;
  columns?: 2 | 3 | 4;
}

export function CardGrid({ children, columns = 3 }: CardGridProps): JSX.Element {
  const minWidth = columns === 2 ? '340px' : columns === 4 ? '220px' : '280px';

  return (
    <div
      className="pai-card-grid"
      style={{
        gridTemplateColumns: `repeat(auto-fit, minmax(${minWidth}, 1fr))`,
      }}
    >
      {children}
    </div>
  );
}

export default CardGrid;
