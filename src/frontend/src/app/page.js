'use client';

import { useState } from 'react';
import Image from 'next/image';
import Link from 'next/link';

import Register from './login/page';

export default function Home() {
  const [isAuth, setIsAuth] = useState(false);

  const handleLogin = () => {
    setIsAuth(true);
  };

  if (!isAuth) {
    return <Register handleLogin={handleLogin} />;
  }

  return (
    <div className="flex flex-col min-h-screen bg-gray-100 font-sans">
      {/* Navbar */}
      <header className="flex justify-between items-center px-5 py-3">
        <div className='justify-start'>
          <Image width={100} height={100} src="/logo_header.png" alt="logo" />
        </div>
      </header>

      {/* Conteúdo Principal */}
      <main className="flex-1 px-8 py-12 text-center justify-center items-center">
        <div className='flex flex-col justify-center items-center px-56 mt-0 py-0'>
          <h1 className="text-7xl font-bold text-gray-800 mb-12 leading-snug text-left">
            Entregas rápidas e seguras de medicamentos com a{' '}
            <span className="text-[#007b9f]">CECIA</span>
          </h1>
        </div>

        <div className="flex flex-wrap gap-6 justify-center">
          {/* Card 1 */}
          <div className="bg-[#e6f7ff] p-6 rounded-lg shadow-md w-72 text-left">
            <h2 className="text-xl font-bold text-gray-800 mb-4">
              Realizar pedido de medicamento
            </h2>
            <p className="text-sm text-gray-600 mb-4">
              É possível fazer pedidos de medicamentos e também permite
              visualizar e acompanhar o status de cada pedido na fila.
            </p>
            <Link
              href="/pages/requests"
              className="text-[#007b9f] font-bold hover:underline"
            >
              Seguir para a página →
            </Link>
          </div>

          {/* Card 2 */}
          <div className="bg-[#ffe6e6] p-6 rounded-lg shadow-md w-72 text-left">
            <h2 className="text-xl font-bold text-gray-800 mb-4">
              Visualização de dashboards
            </h2>
            <p className="text-sm text-gray-600 mb-4">
              Visualização de gráficos que mostram o desempenho do robô,
              inclusão de entregas, e outras informações úteis e interessantes.
            </p>
            <a
              href="#"
              className="text-[#007b9f] font-bold hover:underline"
            >
              Seguir para a página →
            </a>
          </div>

          {/* Card 3 */}
          <div className="bg-[#fff9e6] p-6 rounded-lg shadow-md w-72 text-left">
            <h2 className="text-xl font-bold text-gray-800 mb-4">
              Gerenciar pontos de Entrega
            </h2>
            <p className="text-sm text-gray-600 mb-4">
              Permite adicionar e configurar novos pontos de entrega, além de
              otimizar a rota para o robô levar os medicamentos.
            </p>
            <a
              href="#"
              className="text-[#007b9f] font-bold hover:underline"
            >
              Seguir para a página →
            </a>
          </div>
        </div>
      </main>
    </div>
  );
}
