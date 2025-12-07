import React from 'react';

interface ImageProps extends React.ImgHTMLAttributes<HTMLImageElement> {
  caption?: string;
}

export function Image({ src, alt, caption, className, ...props }: ImageProps) {
  return (
    <figure className="my-6">
      <img 
        src={src} 
        alt={alt || caption} 
        className={`w-full h-auto rounded-lg shadow-md ${className || ''}`}
        {...props} 
      />
      {caption && (
        <figcaption className="text-center text-sm text-muted-foreground mt-2">
          {caption}
        </figcaption>
      )}
    </figure>
  );
}
